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
#include "exss_parser.h"
#include "lbr_decoder.h"

#include "lbr_bitstream.h"
#include "lbr_tables.h"
#include "lbr_huffman.h"

#include "fixed_math.h"

enum LBRHeader {
    LBR_HEADER_SYNC_ONLY    = 1,
    LBR_HEADER_DECODER_INIT = 2
};

enum LBRFlags {
    LBR_FLAG_24_BIT             = 0x01,
    LBR_FLAG_LFE_PRESENT        = 0x02,
    LBR_FLAG_BAND_LIMIT_2_3     = 0x04,
    LBR_FLAG_BAND_LIMIT_1_2     = 0x08,
    LBR_FLAG_BAND_LIMIT_1_3     = 0x0c,
    LBR_FLAG_BAND_LIMIT_1_4     = 0x10,
    LBR_FLAG_BAND_LIMIT_1_8     = 0x18,
    LBR_FLAG_BAND_LIMIT_NONE    = 0x14,
    LBR_FLAG_BAND_LIMIT_MASK    = 0x1c,
    LBR_FLAG_DMIX_STEREO        = 0x20,
    LBR_FLAG_DMIX_MULTI_CH      = 0x40
};

enum LBRChunks {
    LBR_CHUNK_NULL              = 0x00,
    LBR_CHUNK_PAD               = 0x01,
    LBR_CHUNK_FRAME             = 0x04,
    LBR_CHUNK_FRAME_NO_CSUM     = 0x06,
    LBR_CHUNK_LFE               = 0x0a,
    LBR_CHUNK_ECS               = 0x0b,
    LBR_CHUNK_RESERVED_1        = 0x0c,
    LBR_CHUNK_RESERVED_2        = 0x0d,
    LBR_CHUNK_SCF               = 0x0e,
    LBR_CHUNK_TONAL             = 0x10,
    LBR_CHUNK_TONAL_GRP_1       = 0x11,
    LBR_CHUNK_TONAL_GRP_2       = 0x12,
    LBR_CHUNK_TONAL_GRP_3       = 0x13,
    LBR_CHUNK_TONAL_GRP_4       = 0x14,
    LBR_CHUNK_TONAL_GRP_5       = 0x15,
    LBR_CHUNK_TONAL_SCF         = 0x16,
    LBR_CHUNK_TONAL_SCF_GRP_1   = 0x17,
    LBR_CHUNK_TONAL_SCF_GRP_2   = 0x18,
    LBR_CHUNK_TONAL_SCF_GRP_3   = 0x19,
    LBR_CHUNK_TONAL_SCF_GRP_4   = 0x1a,
    LBR_CHUNK_TONAL_SCF_GRP_5   = 0x1b,
    LBR_CHUNK_RES_GRID_LR       = 0x30,
    LBR_CHUNK_RES_GRID_LR_LAST  = 0x3f,
    LBR_CHUNK_RES_GRID_HR       = 0x40,
    LBR_CHUNK_RES_GRID_HR_LAST  = 0x4f,
    LBR_CHUNK_RES_TS_1          = 0x50,
    LBR_CHUNK_RES_TS_1_LAST     = 0x5f,
    LBR_CHUNK_RES_TS_2          = 0x60,
    LBR_CHUNK_RES_TS_2_LAST     = 0x6f,
    LBR_CHUNK_EXTENSION         = 0x7f
};

static int parse_lfe_chunk_24(struct lbr_decoder *lbr)
{
    int ps;

    ps  = bits2_get(&lbr->bits,  8);
    ps |= bits2_get(&lbr->bits, 16) << 8;
    if (ps &  0x800000) {
        ps &= 0x7fffff;
        ps = -ps;
    }

    float value = ps * (1.0 / 0x7fffff);

    int step_i = bits2_get(&lbr->bits, 8);
    if (step_i > 143)
        return -1;

    float step = lfe_step_size_24[step_i];

    for (int i = 0; i < 64; i++) {
        int code = bits2_get(&lbr->bits, 6);

        float delta = step * 0.03125;
        if (code & 16)
            delta += step;
        if (code & 8)
            delta += step * 0.5;
        if (code & 4)
            delta += step * 0.25;
        if (code & 2)
            delta += step * 0.125;
        if (code & 1)
            delta += step * 0.0625;

        if (code & 32) {
            value -= delta;
            if (value < -3.0)
                value = -3.0;
        } else {
            value += delta;
            if (value > 3.0)
                value = 3.0;
        }

        step_i += lfe_delta_index_24[code & 31];
        if (step_i < 0)
            step_i = 0;
        else if (step_i > 143)
            step_i = 143;

        step = lfe_step_size_24[step_i];
        lbr->lfe_data[i] = value;
    }

    return 0;
}

static int parse_lfe_chunk_16(struct lbr_decoder *lbr)
{
    int ps;

    ps = bits2_get(&lbr->bits, 16);
    if (ps &  0x8000) {
        ps &= 0x7fff;
        ps = -ps;
    }

    float value = ps * (1.0 / 0x7fff);

    int step_i = bits2_get(&lbr->bits, 8);
    if (step_i > 100)
        return -1;

    float step = lfe_step_size_16[step_i];

    for (int i = 0; i < 64; i++) {
        int code = bits2_get(&lbr->bits, 4);

        float delta = step * 0.125;
        if (code & 4)
            delta += step;
        if (code & 2)
            delta += step * 0.5;
        if (code & 1)
            delta += step * 0.25;

        if (code & 8) {
            value -= delta;
            if (value < -3.0)
                value = -3.0;
        } else {
            value += delta;
            if (value > 3.0)
                value = 3.0;
        }

        step_i += lfe_delta_index_16[code & 7];
        if (step_i < 0)
            step_i = 0;
        else if (step_i > 100)
            step_i = 100;

        step = lfe_step_size_16[step_i];
        lbr->lfe_data[i] = value;
    }

    return 0;
}

static int parse_ecs_chunk(struct lbr_decoder *lbr)
{
    if (lbr->bits.count < 14)
        return 0;

    int start_sb = bits2_get(&lbr->bits, 7);
    int end_sb = bits2_get(&lbr->bits, 7);

    for (int i = 0; i < lbr->nchannels * lbr->nsubbands * 4; i++) {
        int sb = (i / 4) & (lbr->nsubbands - 1);
        if (lbr->bits.count < 2)
            break;
        if (sb >= start_sb && sb < end_sb && bits2_get1(&lbr->bits)) {
            lbr->can_replace_ch_1[i >> 5] |= 1U << (i & 31);
            if (bits2_get1(&lbr->bits))
                lbr->can_replace_ch_2[i >> 5] |= 1U << (i & 31);
        }
    }

    return 0;
}

static int parse_scf_chunk(struct lbr_decoder *lbr)
{
    if (lbr->bits.count < 36)
        return 0;

    for (int sb = 0; sb < 6; sb++)
        lbr->tonal_scf[sb] = bits2_get(&lbr->bits, 6);

    return 0;
}

static int parse_tonal(struct lbr_decoder *lbr, int group)
{
    int amp[LBR_CHANNELS];
    int phs[LBR_CHANNELS];
    unsigned int diff;

    for (int sf = 0; sf < 1 << group; sf += diff ? 8 : 1) {
        int sf_idx = ((lbr->framenum << group) + sf) & 31;
        lbr->tonal_bounds[group][sf_idx][0] = lbr->ntones;

        for (int freq = 1;; freq++) {
            diff = bits2_get_vlc(&lbr->bits, huff_tnl_grp[group], huff_tnl_size[group]);
            if (diff >= dca_countof(fst_amp))
                return -1;

            diff = bits2_getz(&lbr->bits, diff >> 2) + fst_amp[diff];
            if (diff <= 1)
                break;

            freq += diff - 2;
            if (freq >> (5 - group) > lbr->nsubbands * 4 - 5)
                return -1;

            int main_ch = bits2_getz(&lbr->bits, bits_for_ch_num[lbr->nchannels - 1]);
            int value = bits2_get_vlc(&lbr->bits, huff_tnl_scf, sizeof(huff_tnl_scf));
            if (value < 0)
                return -1;

            value += lbr->tonal_scf[freq_to_sf[freq >> (7 - group)]];
            value += lbr->freq_range - 2;
            if (value < 0 || value > 63)
                return -1;

            amp[main_ch] = value;
            phs[main_ch] = bits2_get(&lbr->bits, 3);

            for (int ch = 0; ch < lbr->nchannels; ch++) {
                if (ch == main_ch)
                    continue;

                if (bits2_get1(&lbr->bits)) {
                    if ((value = bits2_get_vlc(&lbr->bits, huff_damp, sizeof(huff_damp))) < 0)
                        return -1;
                    value = amp[main_ch] - value;
                    if (value < 0 || value > 63)
                        value = 0;
                    amp[ch] = value;

                    if ((value = bits2_get_vlc(&lbr->bits, huff_dph, sizeof(huff_dph))) < 0)
                        return -1;
                    phs[ch] = (phs[main_ch] - value) & 7;
                } else {
                    amp[ch] = 0;
                    phs[ch] = 0;
                }
            }

            struct lbr_tone *t = &lbr->tones[lbr->ntones];
            lbr->ntones = (lbr->ntones + 1) & (LBR_TONES - 1);

            t->x_freq = freq >> (5 - group);
            t->f_delt = (freq & ((1 << (5 - group)) - 1)) << group;
            t->ph_rot = 256 - (t->x_freq & 1) * 128 - t->f_delt * 4;

            value = ph0_shift[(t->x_freq & 3) * 2 + (freq & 1)];
            value -= (t->ph_rot << (5 - group)) - t->ph_rot;

            for (int ch = 0; ch < lbr->nchannels; ch++) {
                t->amp[ch] = amp[ch];
                t->phs[ch] = 128 - phs[ch] * 32 + value;
            }
        }

        lbr->tonal_bounds[group][sf_idx][1] = lbr->ntones;
    }

    return 0;
}

static int parse_scale_factors(struct lbr_decoder *lbr, uint8_t *scf)
{
    int sf = 0, prev, next, dist;

    if (lbr->bits.count < 20)
        return 0;
    if ((prev = bits2_get_vlc(&lbr->bits, huff_fst_rsd_amp, sizeof(huff_fst_rsd_amp))) < 0)
        return -1;

    scf[sf] = prev;

    while (true) {
        if (lbr->bits.count < 20)
            return 0;
        if ((dist = bits2_get_vlc(&lbr->bits, huff_rsd_apprx, sizeof(huff_rsd_apprx))) < 0)
            return -1;
        dist++;
        if (sf + dist > 7)
            return -1;

        if (lbr->bits.count < 20)
            return 0;
        if ((next = bits2_get_vlc(&lbr->bits, huff_rsd_amp, sizeof(huff_rsd_amp))) < 0)
            return -1;

        if (next & 1)
            next = prev + ((next + 1) >> 1);
        else
            next = prev - ( next      >> 1);

        switch (dist) {
        case 2:
            if (next > prev)
                scf[sf + 1] = prev + ((next - prev) >> 1);
            else
                scf[sf + 1] = prev - ((prev - next) >> 1);
            break;

        case 4:
            if (next > prev) {
                scf[sf + 1] = prev + ( (next - prev)      >> 2);
                scf[sf + 2] = prev + ( (next - prev)      >> 1);
                scf[sf + 3] = prev + (((next - prev) * 3) >> 2);
            } else {
                scf[sf + 1] = prev - ( (prev - next)      >> 2);
                scf[sf + 2] = prev - ( (prev - next)      >> 1);
                scf[sf + 3] = prev - (((prev - next) * 3) >> 2);
            }
            break;

        default:
            for (int i = 1; i < dist; i++)
                scf[sf + i] = prev + (next - prev) * i / dist;
            break;
        }

        scf[sf += dist] = next;
        if (sf == 7)
            break;

        prev = next;
    }

    return 0;
}

static int parse_st_code(struct bitstream2 *bits, int min_v)
{
    int v;

    if ((v = bits2_get_vlc(bits, huff_st_grid, sizeof(huff_st_grid))) < 0)
        return -1;

    v += min_v;
    if (v & 1)
        v = 16 + (v >> 1);
    else
        v = 16 - (v >> 1);

    if (v > 33)
        v = 16;
    return v;
}

static int parse_grid_1_chunk(struct lbr_decoder *lbr, int ch1, int ch2)
{
    int ch, sb, sf, value;

    if (lbr->bits.count < 20)
        return 0;

    // Scale factors
    int nsubbands = scf_to_grid_1[lbr->nsubbands - 1] + 1;
    for (sb = 2; sb < nsubbands; sb++) {
        if (parse_scale_factors(lbr, lbr->grid_1_scf[ch1][sb]) < 0)
            return -1;
        if (ch1 != ch2) {
            if (grid_1_to_scf[sb] < lbr->min_mono_subband) {
                if (parse_scale_factors(lbr, lbr->grid_1_scf[ch2][sb]) < 0)
                    return -1;
            } else {
                memset(lbr->grid_1_scf[ch2][sb], 0, sizeof(lbr->grid_1_scf[0][0]));
            }
        }
    }

    if (lbr->bits.count < 20)
        return 0;

    // Average values for third grid
    for (sb = 0; sb < lbr->nsubbands - 4; sb++) {
        if ((value = bits2_get_vlc(&lbr->bits, huff_avg_g3, sizeof(huff_avg_g3))) < 0)
            return -1;
        lbr->grid_3_avg[ch1][sb] = value - 16;
        if (ch1 != ch2) {
            if (sb + 4 < lbr->min_mono_subband) {
                if ((value = bits2_get_vlc(&lbr->bits, huff_avg_g3, sizeof(huff_avg_g3))) < 0)
                    return -1;
                lbr->grid_3_avg[ch2][sb] = value - 16;
            } else {
                lbr->grid_3_avg[ch2][sb] = lbr->grid_3_avg[ch1][sb];
            }
        }
    }

    if (lbr->bits.count < 8)
        return 0;

    // Stereo image for partial mono mode
    if (ch1 != ch2) {
        int min_v[2];

        min_v[0] = bits2_get(&lbr->bits, 4);
        min_v[1] = bits2_get(&lbr->bits, 4);

        int nsubbands = (lbr->nsubbands - lbr->min_mono_subband + 3) / 4;
        for (sb = 0; sb < nsubbands; sb++) {
            for (ch = ch1; ch <= ch2; ch++) {
                for (sf = 1; sf <= 4; sf++) {
                    if ((value = parse_st_code(&lbr->bits, min_v[ch - ch1])) < 0)
                        return -1;
                    lbr->part_stereo[ch][sb][sf] = value;
                }
            }
        }
        lbr->part_stereo_pres |= 1 << ch1;
    }

    if (lbr->bits.count < 8)
        return 0;

    // Low resolution spatial information
    if (ch1 == 0) {
        int nsubbands = (lbr->nsubbands + 3) / 4;
        for (ch = 2; ch < lbr->nchannels; ch++) {
            if (lbr->bits.count < 8)
                break;
            int min_v = bits2_get(&lbr->bits, 4);
            for (sb = 0; sb < nsubbands; sb++) {
                for (sf = 1; sf <= 4; sf++) {
                    if ((value = parse_st_code(&lbr->bits, min_v)) < 0)
                        return -1;
                    lbr->spatial_info[ch - 2][sb][sf] = value;
                }
            }
            lbr->spatial_info_pres |= 1 << ch;
        }
    }

    return 0;
}

static int parse_grid_1_sec_ch(struct lbr_decoder *lbr, int ch2)
{
    int value;

    if (lbr->bits.count < 20)
        return 0;

    // Scale factors
    int nsubbands = scf_to_grid_1[lbr->nsubbands - 1] + 1;
    for (int sb = 2; sb < nsubbands; sb++) {
        if (grid_1_to_scf[sb] >= lbr->min_mono_subband)
            if (parse_scale_factors(lbr, lbr->grid_1_scf[ch2][sb]) < 0)
                return -1;
    }

    if (lbr->bits.count < 20)
        return 0;

    // Average values for third grid
    for (int sb = 0; sb < lbr->nsubbands - 4; sb++) {
        if (sb + 4 >= lbr->min_mono_subband) {
            if ((value = bits2_get_vlc(&lbr->bits, huff_avg_g3, sizeof(huff_avg_g3))) < 0)
                return -1;
            lbr->grid_3_avg[ch2][sb] = value - 16;
        }
    }

    return 0;
}

static int parse_grid_code(struct bitstream2 *bits, const uint8_t *table, int n)
{
    int v = table[bits2_peek(bits, n)];
    bits2_skip(bits, v >> 5);
    return v & 31;
}

static int parse_grid_3_code(struct bitstream2 *bits)
{
    int v;

    if ((v = parse_grid_code(bits, grid_3_codes_1, 5)) < 31)
        return v;
    if ((v = parse_grid_code(bits, grid_3_codes_2, 3)) < 31)
        return v;
    if ((v = parse_grid_code(bits, grid_3_codes_3, 4)) < 31)
        return v;

    v = bits2_get(bits, bits2_get(bits, 3) + 1);
    if (v > 56)
        v = 16;
    return v;
}

static void parse_grid_3(struct lbr_decoder *lbr, int ch1, int ch2, int sb, bool flag)
{
    for (int ch = ch1; ch <= ch2; ch++) {
        if ((ch != ch1 && sb + 4 >= lbr->min_mono_subband) != flag)
            continue;

        if (lbr->grid_3_pres[ch] & (1U << sb))
            continue;

        for (int i = 0; i < 8; i++) {
            if (lbr->bits.count < 20)
                return;
            lbr->grid_3_scf[ch][sb][i] = parse_grid_3_code(&lbr->bits) - 16;
        }

        lbr->grid_3_pres[ch] |= 1U << sb;
    }
}

static float lbr_rand(struct lbr_decoder *lbr, int sb)
{
    lbr->lbr_rand = 1103515245U * lbr->lbr_rand + 12345U;
    return lbr->lbr_rand * lbr->sb_scf[sb];
}

static int parse_ts(struct lbr_decoder *lbr, int ch1, int ch2,
                    int start_sb, int end_sb, bool flag)
{
    int i, j, sb_reorder;

    for (int sb = start_sb; sb < end_sb; sb++) {
        if (lbr->bits.count < 28) {
            lbr->bits.count = 0;
            break;
        }

        if (sb < 6) {
            sb_reorder = sb;
        } else if (flag && sb < lbr->max_mono_subband) {
            sb_reorder = lbr->sb_indices[sb];
        } else {
            sb_reorder = bits2_get(&lbr->bits, lbr->freq_range + 3);
            if (sb_reorder < 6)
                sb_reorder = 6;
            lbr->sb_indices[sb] = sb_reorder;
        }
        if (sb_reorder >= lbr->nsubbands)
            return -1;

        if (sb == 12) {
            for (int sb = 0; sb < lbr->g3_avg_only_start_sb - 4; sb++)
                parse_grid_3(lbr, ch1, ch2, sb, flag);
        } else if (sb < 12 && sb_reorder >= 4) {
            parse_grid_3(lbr, ch1, ch2, sb_reorder - 4, flag);
        }

        if (lbr->bits.count < 20) {
            lbr->bits.count = 0;
            break;
        }

        if (ch1 != ch2) {
            if (!flag || sb_reorder >= lbr->max_mono_subband)
                lbr->sec_ch_sbms[ch1 / 2][sb_reorder] = bits2_get(&lbr->bits, 8);
            if (flag && sb_reorder >= lbr->min_mono_subband)
                lbr->sec_ch_lrms[ch1 / 2][sb_reorder] = bits2_get(&lbr->bits, 8);
        }

        int _ch1 = ch1;
        int _ch2 = ch2;
        if (sb < lbr->max_mono_subband && sb_reorder >= lbr->min_mono_subband) {
            if (flag && ch1 == ch2)
                continue;
            if (flag)
                _ch1 = ch2;
            else
                _ch2 = ch1;
        }

        for (int ch = _ch1; ch <= _ch2; ch++) {
            if (lbr->bits.count < 32) {
                lbr->bits.count = 0;
                break;
            }

            int coding_method = bits2_get1(&lbr->bits);
            float *samples = &lbr->time_samples[ch][sb_reorder][LBR_TIME_HISTORY];
            int code;

            switch (lbr->quant_levels[ch1 / 2][sb]) {
            case 1:
                for (i = 0; i < LBR_TIME_SAMPLES / 8; i++, samples += 8) {
                    if (lbr->bits.count >= 8) {
                        code = bits2_get(&lbr->bits, 8);
                        for (j = 0; j < 8; j++)
                            samples[j] = residual_level_2a[(code >> j) & 1];
                    } else {
                        for (j = 0; j < 8; j++)
                            samples[j] = lbr_rand(lbr, sb_reorder);
                    }
                }
                break;

            case 2:
                if (coding_method) {
                    for (i = 0; i < LBR_TIME_SAMPLES; i++) {
                        if (lbr->bits.count < 2)
                            samples[i] = lbr_rand(lbr, sb_reorder);
                        else if (bits2_get1(&lbr->bits))
                            samples[i] = residual_level_2b[bits2_get1(&lbr->bits)];
                        else
                            samples[i] = 0;
                    }
                    break;
                }

                for (i = 0; i < (LBR_TIME_SAMPLES + 4) / 5; i++, samples += 5) {
                    if (lbr->bits.count >= 8) {
                        code = bits2_get(&lbr->bits, 8);
                        if (code > 242)
                            code = 121;
                    } else {
                        code = 121;
                    }

                    code = residual_pack_5_in_8[code];

                    for (j = 0; j < 5; j++)
                        samples[j] = residual_level_3[(code >> j * 2) & 3];
                }
                break;

            case 3:
                for (i = 0; i < (LBR_TIME_SAMPLES + 2) / 3; i++, samples += 3) {
                    if (lbr->bits.count >= 7) {
                        code = bits2_get(&lbr->bits, 7);
                        if (code > 124)
                            code = 62;
                    } else {
                        code = 62;
                    }

                    for (j = 0; j < 3; j++)
                        samples[j] = residual_level_5[residual_pack_3_in_7[code][j]];
                }
                break;

            case 4:
                for (i = 0; i < LBR_TIME_SAMPLES; i++) {
                    if (lbr->bits.count >= 6) {
                        code = bits2_peek(&lbr->bits, 6);
                        samples[i] = residual_level_8[residual_code_val[code]];
                        bits2_skip(&lbr->bits, residual_code_len[code]);
                    } else {
                        samples[i] = lbr_rand(lbr, sb_reorder);
                    }
                }
                break;

            case 5:
                for (i = 0; i < LBR_TIME_SAMPLES; i++) {
                    if (lbr->bits.count >= 4)
                        samples[i] = residual_level_16[bits2_get(&lbr->bits, 4)];
                    else
                        samples[i] = lbr_rand(lbr, sb_reorder);
                }
                break;

            default:
                return -1;
            }

            lbr->ch_pres[ch] |= 1U << sb_reorder;
        }
    }

    return 0;
}

static void convert_lpc(float *coeff, const int *codes, const float *table)
{
    for (int i = 0; i < 8; i++) {
        float rc = table[codes[i]];
        for (int j = 0; j < (i + 1) / 2; j++) {
            float tmp1 = coeff[    j    ];
            float tmp2 = coeff[i - j - 1];
            coeff[    j    ] = tmp1 + rc * tmp2;
            coeff[i - j - 1] = tmp2 + rc * tmp1;
        }
        coeff[i] = rc;
    }
}

static int parse_lpc(struct lbr_decoder *lbr, int ch1, int ch2, int start_sb, int end_sb)
{
    int f = lbr->framenum & 1;
    int codes[16];

    for (int sb = start_sb; sb < end_sb; sb++) {
        int ncodes = 8 * (1 + (sb < 2));
        for (int ch = ch1; ch <= ch2; ch++) {
            if (lbr->bits.count < 4 * ncodes) {
                memset(lbr->lpc_coeff[ch][sb][f], 0, sizeof(lbr->lpc_coeff[0][0][0]));
                lbr->bits.count = 0;
                return 0;
            }
            for (int i = 0; i < ncodes; i++)
                codes[i] = bits2_get(&lbr->bits, 4);
            for (int i = 0; i < ncodes / 8; i++)
                convert_lpc(lbr->lpc_coeff[ch][sb][f][i], &codes[i * 8], lbr->lpc_tab);
        }
    }

    return 0;
}

static int parse_high_res_grid(struct lbr_decoder *lbr, int ch1, int ch2)
{
    if (lbr->bits.count < 8)
        return -1;

    // Quantizer profile
    int profile = bits2_get(&lbr->bits, 8);
    int ol = (profile >> 3) & 7;
    int st = profile >> 6;
    int sb, max_sb = profile & 7;

    // Calculate quantization levels
    int quant_levels[LBR_SUBBANDS];
    for (sb = 0; sb < lbr->nsubbands; sb++) {
        int f = sb * lbr->limited_rate / lbr->nsubbands;
        int a = 18000 / (12 * f / 1000 + 100 + 40 * st) + 20 * ol;
        if (a <= 95)
            quant_levels[sb] = 1;
        else if (a <= 140)
            quant_levels[sb] = 2;
        else if (a <= 180)
            quant_levels[sb] = 3;
        else if (a <= 230)
            quant_levels[sb] = 4;
        else
            quant_levels[sb] = 5;
    }

    // Reorder quantization levels for lower subbands
    for (sb = 0; sb < 8; sb++)
        lbr->quant_levels[ch1 / 2][sb] = quant_levels[sb_reorder[max_sb][sb]];
    for (; sb < lbr->nsubbands; sb++)
        lbr->quant_levels[ch1 / 2][sb] = quant_levels[sb];

    // LPC for the first two subbands
    if (parse_lpc(lbr, ch1, ch2, 0, 2) < 0)
        return -1;

    // Time-samples for the first two subbands of main channel
    if (parse_ts(lbr, ch1, ch2, 0, 2, false) < 0)
        return -1;

    // First two bands of the first grid
    for (sb = 0; sb < 2; sb++)
        for (int ch = ch1; ch <= ch2; ch++)
            if (parse_scale_factors(lbr, lbr->grid_1_scf[ch][sb]) < 0)
                return -1;

    return 0;
}

static int parse_grid_2_code(struct bitstream2 *bits)
{
    int v;

    if ((v = parse_grid_code(bits, grid_2_codes_1, 5)) < 31)
        return v;
    if ((v = parse_grid_code(bits, grid_2_codes_2, 4)) < 31)
        return v;
    if ((v = parse_grid_code(bits, grid_2_codes_3, 5)) < 31)
        return v;

    v = bits2_get(bits, bits2_get(bits, 3) + 1);
    if (v > 56)
        v = 0;
    return v;
}

static int parse_grid_2(struct lbr_decoder *lbr, int ch1, int ch2,
                        int start_sb, int end_sb, bool flag)
{
    int nsubbands2 = scf_to_grid_2[lbr->nsubbands - 1] + 1;
    if (end_sb > nsubbands2)
        end_sb = nsubbands2;

    for (int sb = start_sb; sb < end_sb; sb++) {
        for (int ch = ch1; ch <= ch2; ch++) {
            if ((ch != ch1 && grid_2_to_scf[sb] >= lbr->min_mono_subband) != flag) {
                if (!flag)
                    memcpy(lbr->grid_2_scf[ch2][sb], lbr->grid_2_scf[ch1][sb], sizeof(lbr->grid_2_scf[0][0]));
                continue;
            }

            uint8_t *g2_scf = lbr->grid_2_scf[ch][sb];
            for (int i = 0; i < 8; i++) {
                if (lbr->bits.count > 1 && bits2_get1(&lbr->bits)) {
                    for (int j = 0; j < 8; j++) {
                        if (lbr->bits.count < 20)
                            break;
                        g2_scf[j] = parse_grid_2_code(&lbr->bits);
                    }
                } else {
                    memset(g2_scf, 0, 8 * sizeof(*g2_scf));
                }
                g2_scf += 8;
            }
        }
    }

    return 0;
}

static int parse_ts1_chunk(struct lbr_decoder *lbr, int ch1, int ch2)
{
    if (parse_lpc(lbr, ch1, ch2, 2, 3) < 0)
        return -1;
    if (parse_ts(lbr, ch1, ch2, 2, 4, false) < 0)
        return -1;
    if (parse_grid_2(lbr, ch1, ch2, 0, 1, false) < 0)
        return -1;
    if (parse_ts(lbr, ch1, ch2, 4, 6, false) < 0)
        return -1;
    return 0;
}

static int parse_ts2_chunk(struct lbr_decoder *lbr, int ch1, int ch2)
{
    if (parse_grid_2(lbr, ch1, ch2, 1, 3, false) < 0)
        return -1;
    if (parse_ts(lbr, ch1, ch2, 6, lbr->max_mono_subband, false) < 0)
        return -1;
    if (ch1 != ch2) {
        if (parse_grid_1_sec_ch(lbr, ch2) < 0)
            return -1;
        if (parse_grid_2(lbr, ch1, ch2, 0, 3, true) < 0)
            return -1;
    }
    if (parse_ts(lbr, ch1, ch2, lbr->min_mono_subband, lbr->nsubbands, true) < 0)
        return -1;
    return 0;
}

static void init_tables(struct lbr_decoder *lbr)
{
    float scale = 256.0 * 0.25;
    int i, j;

    switch (lbr->flags & LBR_FLAG_BAND_LIMIT_MASK) {
    case LBR_FLAG_BAND_LIMIT_1_2:
        scale *= sqrt(2.0);
        break;
    case LBR_FLAG_BAND_LIMIT_1_4:
        scale *= sqrt(4.0);
        break;
    case LBR_FLAG_BAND_LIMIT_1_8:
        scale *= sqrt(8.0);
        break;
    }

    for (i = 0; i < 256; i++) {
        for (j = 0; j < 128; j++)
            lbr->cos_mod[i][j] = scale * cos(M_PI * (2 * i + 129) * (2 * j + 1) / 512);
        lbr->sin_tab[i] = cos(M_PI * i / 128);
    }

    for (i = 0; i < 16; i++)
        lbr->lpc_tab[i] = sin((i - 8) * (M_PI / ((i < 8) ? 17 : 15)));

    int br_per_ch = lbr->bit_rate_scaled / lbr->nchannels;

    if (br_per_ch < 14000)
        scale = 0.85;
    else if (br_per_ch < 32000)
        scale = (br_per_ch - 14000) * (1.0 / 120000) + 0.85;
    else
        scale = 1.0;

    scale *= 1.0 / INT_MAX;

    for (i = 0; i < LBR_SUBBANDS; i++) {
        if (i < 2)
            lbr->sb_scf[i] = 0;
        else if (i < 5)
            lbr->sb_scf[i] = (i - 1) * 0.25 * 0.785 * scale;
        else
            lbr->sb_scf[i] = 0.785 * scale;
    }
}

static int parse_decoder_init(struct lbr_decoder *lbr, struct bytestream *bytes)
{
    int old_rate = lbr->sample_rate;
    int old_flags = lbr->flags;

    // Sample rate of LBR audio
    unsigned int code = bytes_get(bytes);
    if (code >= dca_countof(sample_rates) || sample_rates[code] != 48000) {
        lbr_err("Invalid LBR sample rate");
        return -DCADEC_EBADDATA;
    }
    lbr->sample_rate = sample_rates[code];

    // LBR speaker mask
    lbr->ch_mask = bytes_get16le(bytes);

    // LBR bitstream version
    if ((bytes_get16le(bytes) & 0xff00) != 0x0800) {
        lbr_err("Unsupported LBR version");
        return -DCADEC_ENOSUP;
    }

    // Flags for LBR decoder initialization
    lbr->flags = bytes_get(bytes);
    if (lbr->flags & (LBR_FLAG_DMIX_STEREO | LBR_FLAG_DMIX_MULTI_CH)) {
        lbr_err("Unsupported embedded downmix");
        return -DCADEC_ENOSUP;
    }

    // Most significant bit rate nibbles
    int bit_rate_hi = bytes_get(bytes);

    // Least significant original bit rate word
    lbr->bit_rate_orig = bytes_get16le(bytes) | ((bit_rate_hi & 0x0F) << 16);

    // Least significant scaled bit rate word
    lbr->bit_rate_scaled = bytes_get16le(bytes) | ((bit_rate_hi & 0xF0) << 12);

    lbr->nchannels = count_chs_for_mask(lbr->ch_mask & ~SPEAKER_PAIR_LFE1);
    if (lbr->nchannels < 1 || lbr->nchannels > LBR_CHANNELS) {
        lbr_err("Invalid LBR channel mask");
        return -DCADEC_EBADDATA;
    }

    switch (lbr->flags & LBR_FLAG_BAND_LIMIT_MASK) {
    case LBR_FLAG_BAND_LIMIT_1_2:
        lbr->limited_rate = lbr->sample_rate / 2;
        break;
    case LBR_FLAG_BAND_LIMIT_1_4:
        lbr->limited_rate = lbr->sample_rate / 4;
        break;
    case LBR_FLAG_BAND_LIMIT_1_8:
        lbr->limited_rate = lbr->sample_rate / 8;
        break;
    case LBR_FLAG_BAND_LIMIT_NONE:
        lbr->limited_rate = lbr->sample_rate;
        break;
    default:
        lbr_err("Invalid LBR band limit");
        return -DCADEC_EBADDATA;
    }

    if (lbr->limited_rate < 14000)
        lbr->freq_range = 0;
    else if (lbr->limited_rate < 28000)
        lbr->freq_range = 1;
    else
        lbr->freq_range = 2;

    if (lbr->bit_rate_orig >= 44000 * (lbr->nchannels + 2))
        lbr->res_profile = 2;
    else if (lbr->bit_rate_orig >= 25000 * (lbr->nchannels + 2))
        lbr->res_profile = 1;
    else
        lbr->res_profile = 0;

    lbr->nsubbands = 8 << lbr->freq_range;

    static const uint16_t freq[3] = { 16000, 18000, 24000 };

    lbr->g3_avg_only_start_sb = lbr->nsubbands * freq[lbr->res_profile] / (lbr->limited_rate / 2);
    if (lbr->g3_avg_only_start_sb > lbr->nsubbands)
        lbr->g3_avg_only_start_sb = lbr->nsubbands;

    lbr->min_mono_subband = lbr->nsubbands *  2000 / (lbr->limited_rate / 2);
    if (lbr->min_mono_subband > lbr->nsubbands)
        lbr->min_mono_subband = lbr->nsubbands;

    lbr->max_mono_subband = lbr->nsubbands * 14000 / (lbr->limited_rate / 2);
    if (lbr->max_mono_subband > lbr->nsubbands)
        lbr->max_mono_subband = lbr->nsubbands;

    if (old_rate != lbr->sample_rate || old_flags != lbr->flags) {
        memset(lbr->part_stereo, 16, sizeof(lbr->part_stereo));
        memset(lbr->spatial_info, 16, sizeof(lbr->spatial_info));

        init_tables(lbr);
    }

    return 0;
}

int lbr_parse(struct lbr_decoder *lbr, uint8_t *data, size_t size, struct exss_asset *asset)
{
    int ret;

    (void)size;

    struct bytestream bytes;
    bytes_init(&bytes, data + asset->lbr_offset, asset->lbr_size);

    // LBR sync word
    if (bytes_get32be(&bytes) != SYNC_WORD_LBR) {
        lbr_err("Invalid LBR sync word");
        return -DCADEC_ENOSYNC;
    }

    // LBR header type
    switch (bytes_get(&bytes)) {
    case LBR_HEADER_SYNC_ONLY:
        if (!lbr->sample_rate) {
            lbr_err("LBR decoder not initialized");
            return -DCADEC_EBADDATA;
        }
        break;
    case LBR_HEADER_DECODER_INIT:
        if ((ret = parse_decoder_init(lbr, &bytes)) < 0) {
            lbr->sample_rate = 0;
            return ret;
        }
        break;
    default:
        lbr_err("Invalid LBR header type");
        return -DCADEC_EBADDATA;
    }

    int chunk_id = bytes_get(&bytes);
    int chunk_len = (chunk_id & 0x80) ? bytes_get16be(&bytes) : bytes_get(&bytes);

    if (bytes.index + chunk_len > bytes.total) {
        lbr_err("Invalid LBR frame chunk size");
        return -DCADEC_EBADREAD;
    }

    switch (chunk_id & 0x7f) {
    case LBR_CHUNK_FRAME: {
        int checksum = bytes_get16be(&bytes);
        uint16_t res = chunk_id;
        res += (chunk_len >> 8) & 0xff;
        res += chunk_len & 0xff;
        for (int i = 0; i < chunk_len - 2; i++)
            res += bytes.data[bytes.index + i];
        if (checksum != res) {
            lbr_err("Invalid LBR checksum");
            return -DCADEC_EBADCRC;
        }
        break;
    }
    case LBR_CHUNK_FRAME_NO_CSUM:
        break;
    default:
        lbr_err("Invalid LBR frame chunk ID");
        return -DCADEC_EBADDATA;
    }

    memset(lbr->can_replace_ch_1, 0, sizeof(lbr->can_replace_ch_1));
    memset(lbr->can_replace_ch_2, 0, sizeof(lbr->can_replace_ch_2));
    memset(lbr->quant_levels, 0, sizeof(lbr->quant_levels));
    memset(lbr->sb_indices, 0xff, sizeof(lbr->sb_indices));
    memset(lbr->sec_ch_sbms, 0, sizeof(lbr->sec_ch_sbms));
    memset(lbr->sec_ch_lrms, 0, sizeof(lbr->sec_ch_lrms));
    memset(lbr->ch_pres, 0, sizeof(lbr->ch_pres));
    memset(lbr->grid_1_scf, 0, sizeof(lbr->grid_1_scf));
    memset(lbr->grid_2_scf, 0, sizeof(lbr->grid_2_scf));
    memset(lbr->grid_3_avg, 0, sizeof(lbr->grid_3_avg));
    memset(lbr->grid_3_scf, 0, sizeof(lbr->grid_3_scf));
    memset(lbr->grid_3_pres, 0, sizeof(lbr->grid_3_pres));
    memset(lbr->tonal_scf, 0, sizeof(lbr->tonal_scf));
    memset(lbr->lfe_data, 0, sizeof(lbr->lfe_data));
    lbr->part_stereo_pres = 0;
    lbr->spatial_info_pres = 0;
    lbr->framenum = (lbr->framenum + 1) & 31;

    for (int ch = 0; ch < lbr->nchannels; ch++) {
        for (int sb = 0; sb < lbr->nsubbands / 4; sb++) {
            lbr->part_stereo[ch][sb][0] = lbr->part_stereo[ch][sb][4];
            lbr->part_stereo[ch][sb][4] = 16;
        }
    }

    for (int ch = 0; ch < lbr->nchannels - 2; ch++) {
        for (int sb = 0; sb < lbr->nsubbands / 4; sb++) {
            lbr->spatial_info[ch][sb][0] = lbr->spatial_info[ch][sb][4];
            lbr->spatial_info[ch][sb][4] = 16;
        }
    }

    for (int group = 0; group < 5; group++) {
        for (int sf = 0; sf < 1 << group; sf++) {
            int sf_idx = ((lbr->framenum << group) + sf) & 31;
            lbr->tonal_bounds[group][sf_idx][0] =
            lbr->tonal_bounds[group][sf_idx][1] = lbr->ntones;
        }
    }

    while (bytes.index < bytes.total) {
        int chunk_id = bytes_get(&bytes);
        int chunk_len = (chunk_id & 0x80) ? bytes_get16be(&bytes) : bytes_get(&bytes);
        int ch1, ch2;

        int chunk_end = bytes.index + chunk_len;
        if (chunk_end > bytes.total)
            chunk_end = bytes.total;

        chunk_id &= 0x7f;

        bits2_init(&lbr->bits, bytes.data + bytes.index, chunk_len);

        ret = 0;
        switch (chunk_id) {
        case LBR_CHUNK_LFE:
            if (lbr->flags & LBR_FLAG_LFE_PRESENT) {
                if (chunk_len >= 52)
                    ret = parse_lfe_chunk_24(lbr);
                else if (chunk_len >= 35)
                    ret = parse_lfe_chunk_16(lbr);
                else
                    lbr_warn("LFE chunk too small");
            }
            break;

        case LBR_CHUNK_ECS:
            ret = parse_ecs_chunk(lbr);
            break;

        case LBR_CHUNK_SCF:
            ret = parse_scf_chunk(lbr);
            break;

        case LBR_CHUNK_TONAL_SCF_GRP_1 ... LBR_CHUNK_TONAL_SCF_GRP_5:
            ret = parse_tonal(lbr, LBR_CHUNK_TONAL_SCF_GRP_5 - chunk_id);
            break;

        case LBR_CHUNK_RES_GRID_LR ... LBR_CHUNK_RES_GRID_LR + LBR_CHANNELS / 2:
            ch1 = 2 * (chunk_id - LBR_CHUNK_RES_GRID_LR);
            ch2 = DCA_MIN(ch1 + 1, lbr->nchannels - 1);
            if (ch1 <= ch2)
                ret = parse_grid_1_chunk(lbr, ch1, ch2);
            break;

        case LBR_CHUNK_RES_GRID_HR ... LBR_CHUNK_RES_GRID_HR + LBR_CHANNELS / 2:
            ch1 = 2 * (chunk_id - LBR_CHUNK_RES_GRID_HR);
            ch2 = DCA_MIN(ch1 + 1, lbr->nchannels - 1);
            if (ch1 <= ch2)
                ret = parse_high_res_grid(lbr, ch1, ch2);
            break;

        case LBR_CHUNK_RES_TS_1 ... LBR_CHUNK_RES_TS_1 + LBR_CHANNELS / 2:
            ch1 = 2 * (chunk_id - LBR_CHUNK_RES_TS_1);
            ch2 = DCA_MIN(ch1 + 1, lbr->nchannels - 1);
            if (ch1 <= ch2)
                ret = parse_ts1_chunk(lbr, ch1, ch2);
            break;

        case LBR_CHUNK_RES_TS_2 ... LBR_CHUNK_RES_TS_2 + LBR_CHANNELS / 2:
            ch1 = 2 * (chunk_id - LBR_CHUNK_RES_TS_2);
            ch2 = DCA_MIN(ch1 + 1, lbr->nchannels - 1);
            if (ch1 <= ch2)
                ret = parse_ts2_chunk(lbr, ch1, ch2);
            break;
        }

        if (ret < 0) {
            lbr_err("Error decoding chunk %#x", chunk_id);
            return -DCADEC_EBADDATA;
        }

        bytes.index = chunk_end;
    }

    return 0;
}

static void decode_grid(struct lbr_decoder *lbr, int ch)
{
    for (int sb = 0; sb < lbr->nsubbands; sb++) {
        int g1_sb = scf_to_grid_1[sb];

        uint8_t *g1_scf_a = lbr->grid_1_scf[ch][g1_sb    ];
        uint8_t *g1_scf_b = lbr->grid_1_scf[ch][g1_sb + 1];

        int w1 = grid_1_weights[g1_sb    ][sb];
        int w2 = grid_1_weights[g1_sb + 1][sb];

        uint8_t *hr_scf = lbr->high_res_scf[ch][sb];

        if (sb < 4) {
            for (int i = 0; i < 8; i++) {
                int scf = w1 * g1_scf_a[i] + w2 * g1_scf_b[i];
                hr_scf[i] = scf >> 7;
            }
        } else {
            int8_t *g3_scf = lbr->grid_3_scf[ch][sb - 4];
            int g3_avg = lbr->grid_3_avg[ch][sb - 4];

            for (int i = 0; i < 8; i++) {
                int scf = w1 * g1_scf_a[i] + w2 * g1_scf_b[i];
                hr_scf[i] = (scf >> 7) - g3_avg - g3_scf[i];
            }
        }
    }
}

static void random_ts(struct lbr_decoder *lbr, int ch)
{
    for (int sb = 0; sb < lbr->nsubbands; sb++) {
        float *samples = &lbr->time_samples[ch][sb][LBR_TIME_HISTORY];
        int i, j;

        if (lbr->ch_pres[ch] & (1U << sb))
            continue;

        if (sb < 2) {
            memset(samples, 0, LBR_TIME_SAMPLES * sizeof(*samples));
        } else if (sb < 10) {
            for (i = 0; i < LBR_TIME_SAMPLES; i++)
                samples[i] = lbr_rand(lbr, sb);
        } else {
            for (i = 0; i < LBR_TIME_SAMPLES / 8; i++, samples += 8) {
                float accum[8] = { 0 };

                for (int sb = 2; sb < 6; sb++) {
                    float *other = &lbr->time_samples[ch][sb][LBR_TIME_HISTORY + i * 8];
                    for (j = 0; j < 8; j++)
                        accum[j] += fabs(other[j]);
                }

                for (j = 0; j < 8; j++)
                    samples[j] = (accum[j] * 0.25 + 0.5) * lbr_rand(lbr, sb);
            }
        }
    }
}

static void predict(float *samples, const float *coeff, int nsamples)
{
    for (int i = 0; i < nsamples; i++) {
        double res = 0.0;
        for (int j = 0; j < 8; j++)
            res += coeff[j] * samples[i - j - 1];
        samples[i] -= res;
    }
}

static void synth_lpc(struct lbr_decoder *lbr, int ch, int sb)
{
    float *samples = &lbr->time_samples[ch][sb][LBR_TIME_HISTORY];
    int f = lbr->framenum & 1;

    if (!(lbr->ch_pres[ch] & (1U << sb)))
        return;

    if (sb < 2) {
        predict(samples,      lbr->lpc_coeff[ch][sb][f^1][1],  16);
        predict(samples + 16, lbr->lpc_coeff[ch][sb][f  ][0],  64);
        predict(samples + 80, lbr->lpc_coeff[ch][sb][f  ][1],  48);
    } else {
        predict(samples,      lbr->lpc_coeff[ch][sb][f^1][0],  16);
        predict(samples + 16, lbr->lpc_coeff[ch][sb][f  ][0], 112);
    }
}

static void filter_ts(struct lbr_decoder *lbr, int ch1, int ch2)
{
    int i, j;

    for (int sb = 0; sb < lbr->nsubbands; sb++) {
        // Scale factors
        for (int ch = ch1; ch <= ch2; ch++) {
            float *samples = &lbr->time_samples[ch][sb][LBR_TIME_HISTORY];
            uint8_t *hr_scf = lbr->high_res_scf[ch][sb];
            if (sb < 4) {
                for (i = 0; i < LBR_TIME_SAMPLES / 16; i++, samples += 16) {
                    unsigned int scf = hr_scf[i];
                    if (scf > 63)
                        scf = 63;
                    for (j = 0; j < 16; j++)
                        samples[j] *= quant_amp[scf];
                }
            } else {
                uint8_t *g2_scf = lbr->grid_2_scf[ch][scf_to_grid_2[sb]];
                for (i = 0; i < LBR_TIME_SAMPLES / 2; i++, samples += 2) {
                    unsigned int scf = hr_scf[i / 8] - g2_scf[i];
                    if (scf > 63)
                        scf = 63;
                    samples[0] *= quant_amp[scf];
                    samples[1] *= quant_amp[scf];
                }
            }
        }

        // Mid-side stereo
        if (ch1 != ch2) {
            float *samples_l = &lbr->time_samples[ch1][sb][LBR_TIME_HISTORY];
            float *samples_r = &lbr->time_samples[ch2][sb][LBR_TIME_HISTORY];
            int ch2_pres = lbr->ch_pres[ch2] & (1U << sb);

            for (i = 0; i < LBR_TIME_SAMPLES / 16; i++) {
                int sbms = (lbr->sec_ch_sbms[ch1 / 2][sb] >> i) & 1;
                int lrms = (lbr->sec_ch_lrms[ch1 / 2][sb] >> i) & 1;

                if (sb >= lbr->min_mono_subband) {
                    if (lrms && ch2_pres) {
                        if (sbms) {
                            for (j = 0; j < 16; j++) {
                                float tmp = samples_l[j];
                                samples_l[j] =  samples_r[j];
                                samples_r[j] = -tmp;
                            }
                        } else {
                            for (j = 0; j < 16; j++) {
                                float tmp = samples_l[j];
                                samples_l[j] =  samples_r[j];
                                samples_r[j] =  tmp;
                            }
                        }
                    } else if (!ch2_pres) {
                        if (sbms && (lbr->part_stereo_pres & (1 << ch1))) {
                            for (j = 0; j < 16; j++)
                                samples_r[j] = -samples_l[j];
                        } else {
                            for (j = 0; j < 16; j++)
                                samples_r[j] =  samples_l[j];
                        }
                    }
                } else if (sbms && ch2_pres) {
                    for (j = 0; j < 16; j++) {
                        float tmp = samples_l[j];
                        samples_l[j] = (tmp + samples_r[j]) * 0.5;
                        samples_r[j] = (tmp - samples_r[j]) * 0.5;
                    }
                }

                samples_l += 16;
                samples_r += 16;
            }
        }

        // Inverse prediciton
        if (sb < 3) {
            synth_lpc(lbr, ch1, sb);
            if (ch1 != ch2)
                synth_lpc(lbr, ch2, sb);
        }
    }
}

static void decode_part_stereo(struct lbr_decoder *lbr, int ch1, int ch2)
{
    for (int ch = ch1; ch <= ch2; ch++) {
        for (int sb = lbr->min_mono_subband; sb < lbr->nsubbands; sb++) {
            if (lbr->ch_pres[ch2] & (1U << sb))
                continue;

            float *samples = &lbr->time_samples[ch][sb][LBR_TIME_HISTORY];
            int pt_sb = (sb - lbr->min_mono_subband) / 4;

            for (int sf = 1; sf <= 4; sf++) {
                float prev = st_coeff[lbr->part_stereo[ch][pt_sb][sf - 1]];
                float next = st_coeff[lbr->part_stereo[ch][pt_sb][sf    ]];

                for (int i = 0; i < 32; i++)
                    samples[i] *= (32 - i) * prev + i * next;

                samples += 32;
            }
        }
    }
}

static void decode_spatial_info(struct lbr_decoder *lbr, int ch1, int ch2)
{
    for (int sb = 0; sb < lbr->nsubbands; sb++) {
        if (lbr->ch_pres[ch2] & (1U << sb))
            continue;

        float *samples_l = &lbr->time_samples[ch1][sb][LBR_TIME_HISTORY];
        float *samples_r = &lbr->time_samples[ch2][sb][LBR_TIME_HISTORY];

        for (int sf = 1; sf <= 4; sf++) {
            float prev = st_coeff[lbr->spatial_info[ch2 - 2][sb / 4][sf - 1]];
            float next = st_coeff[lbr->spatial_info[ch2 - 2][sb / 4][sf    ]];

            for (int i = 0; i < 32; i++)
                samples_r[i] = ((32 - i) * prev + i * next) * samples_l[i];

            samples_l += 32;
            samples_r += 32;
        }
    }
}

static void synth_tones(struct lbr_decoder *lbr, int ch, float *values,
                        int group, int group_sf, int synth_idx)
{
    if (synth_idx < 0)
        return;

    int start =  lbr->tonal_bounds[group][group_sf][0];
    int count = (lbr->tonal_bounds[group][group_sf][1] - start) & (LBR_TONES - 1);

    for (int i = 0; i < count; i++) {
        struct lbr_tone *t = &lbr->tones[(start + i) & (LBR_TONES - 1)];

        if (t->amp[ch]) {
            float amp = synth_env[synth_idx] * quant_amp[t->amp[ch]];
            float s = amp * lbr->sin_tab[(t->phs[ch]     ) & 255];
            float c = amp * lbr->sin_tab[(t->phs[ch] + 64) & 255];
            const float *cf = corr_cf[t->f_delt];
            int x_freq = t->x_freq;

            switch (x_freq) {
            case 0:
                goto p0;
            case 1:
                values[3] += cf[0] * -c;
                values[2] += cf[1] *  s;
                values[1] += cf[2] *  c;
                values[0] += cf[3] * -s;
                goto p1;
            case 2:
                values[2] += cf[0] * -c;
                values[1] += cf[1] *  s;
                values[0] += cf[2] *  c;
                goto p2;
            case 3:
                values[1] += cf[0] * -c;
                values[0] += cf[1] *  s;
                goto p3;
            case 4:
                values[0] += cf[0] * -c;
                goto p4;
            }

            values[x_freq - 5] += cf[ 0] * -c;
        p4: values[x_freq - 4] += cf[ 1] *  s;
        p3: values[x_freq - 3] += cf[ 2] *  c;
        p2: values[x_freq - 2] += cf[ 3] * -s;
        p1: values[x_freq - 1] += cf[ 4] * -c;
        p0: values[x_freq    ] += cf[ 5] *  s;
            values[x_freq + 1] += cf[ 6] *  c;
            values[x_freq + 2] += cf[ 7] * -s;
            values[x_freq + 3] += cf[ 8] * -c;
            values[x_freq + 4] += cf[ 9] *  s;
            values[x_freq + 5] += cf[10] *  c;
        }

        t->phs[ch] += t->ph_rot;
    }
}

static void base_func_synth(struct lbr_decoder *lbr, int ch, float *values, int sf)
{
    for (int group = 0; group < 5; group++) {
        int group_sf = (lbr->framenum << group) + ((sf - 22) >> (5 - group));
        int synth_idx = ((((sf - 22) & 31) << group) & 31) + (1 << group) - 1;

        synth_tones(lbr, ch, values, group, (group_sf - 1) & 31, 30 - synth_idx);
        synth_tones(lbr, ch, values, group, (group_sf    ) & 31,      synth_idx);
    }
}

#define SW0     0.022810893
#define SW1     0.41799772
#define SW2     0.9084481
#define SW3     0.99973983

#define C1      0.068974845
#define C2      0.34675997
#define C3      0.29396889
#define C4      0.19642374

#define AL1     0.30865827
#define AL2     0.038060233

static inline int convert(float a)
{
    return clip23(lrintf(a));
}

static void imdct128(struct lbr_decoder *lbr, float *output, const float *input)
{
    for (int i = 0; i < 256; i++) {
        double res = 0.0;
        for (int j = 0; j < 128; j++)
            res += input[j] * lbr->cos_mod[i][j];
        output[i] = res;
    }
}

static void transform_channel(struct lbr_decoder *lbr, int ch)
{
    float values[LBR_SUBBANDS][4];
    float _values[LBR_SUBBANDS * 2][4];
    int *output = lbr->channel_buffer[ch];
    int i, sf, nsubbands = lbr->nsubbands;

    for (sf = 0; sf < LBR_TIME_SAMPLES / 4; sf++) {
        // Short window and 8 point forward MDCT
        for (i = 0; i < nsubbands; i++) {
            float *samples = &lbr->time_samples[ch][i][LBR_TIME_HISTORY + sf * 4];

            float a = samples[-4] * SW0 - samples[-1] * SW3;
            float b = samples[-3] * SW1 - samples[-2] * SW2;
            float c = samples[ 2] * SW1 + samples[ 1] * SW2;
            float d = samples[ 3] * SW0 + samples[ 0] * SW3;

            values[i][0] = C1 * b - C2 * c + C4 * a - C3 * d;
            values[i][1] = C1 * d - C2 * a - C4 * b - C3 * c;
            values[i][2] = C3 * b + C2 * d - C4 * c + C1 * a;
            values[i][3] = C3 * a - C2 * b + C4 * d - C1 * c;
        }

        // Aliasing cancellation for high frequencies
        for (i = 12; i < nsubbands - 1; i++) {
            float a = values[i  ][3] * AL1;
            float b = values[i+1][0] * AL1;
            values[i  ][3] += b - a;
            values[i+1][0] -= b + a;
            a = values[i  ][2] * AL2;
            b = values[i+1][1] * AL2;
            values[i  ][2] += b - a;
            values[i+1][1] -= b + a;
        }

        // Clear inactive subbands
        if (nsubbands < LBR_SUBBANDS)
            memset(values[nsubbands], 0, (LBR_SUBBANDS - nsubbands) * sizeof(values[0]));

        base_func_synth(lbr, ch, values[0], sf);

        imdct128(lbr, _values[0], values[0]);

        // Long window and overlap-add
        const float *w1 = &long_window[      0];
        const float *w2 = &long_window[128 - 4];
        float *history = lbr->imdct_history[ch];
        for (i = 0; i < LBR_SUBBANDS; i++) {
            output[0] = convert(w1[0] * _values[i][0] + history[0]);
            output[1] = convert(w1[1] * _values[i][1] + history[1]);
            output[2] = convert(w1[2] * _values[i][2] + history[2]);
            output[3] = convert(w1[3] * _values[i][3] + history[3]);

            history[0] = w2[3] * _values[LBR_SUBBANDS + i][0];
            history[1] = w2[2] * _values[LBR_SUBBANDS + i][1];
            history[2] = w2[1] * _values[LBR_SUBBANDS + i][2];
            history[3] = w2[0] * _values[LBR_SUBBANDS + i][3];

            output  += 4;
            history += 4;
            w1 += 4;
            w2 -= 4;
        }
    }

    // Update history for LPC and forward MDCT
    for (i = 0; i < nsubbands; i++) {
        float *samples = lbr->time_samples[ch][i];
        memcpy(samples, samples + LBR_TIME_SAMPLES, LBR_TIME_HISTORY * sizeof(*samples));
    }
}

#define LFE_IIR_SCALE   (64 * 0x7fffff * 0.0000078265894)

static void interpolate_lfe(struct lbr_decoder *lbr)
{
    int *output = lbr->output_samples[SPEAKER_LFE1];

    for (int i = 0; i < 64; i++) {
        float res1 = lbr->lfe_data[i] * LFE_IIR_SCALE;
        float res2;

        for (int j = 0; j < 64; j++) {
            for (int k = 0; k < 5; k++) {
                float tmp1 = lbr->lfe_history[k][0];
                float tmp2 = lbr->lfe_history[k][1];

                res2 = tmp1 * lfe_iir[k][0] + tmp2 * lfe_iir[k][1] + res1;
                res1 = tmp1 * lfe_iir[k][2] + tmp2 * lfe_iir[k][3] + res2;

                lbr->lfe_history[k][0] = tmp2;
                lbr->lfe_history[k][1] = res2;
            }

            *output++ = convert(res1);
            res1 = 0.0;
        }
    }
}

int lbr_filter(struct lbr_decoder *lbr)
{
    for (int pair = 0; pair < (lbr->nchannels + 1) / 2; pair++) {
        int ch1 = pair * 2;
        int ch2 = DCA_MIN(ch1 + 1, lbr->nchannels - 1);

        decode_grid(lbr, ch1);
        if (ch1 != ch2)
            decode_grid(lbr, ch2);

        random_ts(lbr, ch1);
        if (ch1 != ch2)
            random_ts(lbr, ch2);

        filter_ts(lbr, ch1, ch2);

        if (ch1 != ch2 && (lbr->part_stereo_pres & (1 << ch1)) && 1)
            decode_part_stereo(lbr, ch1, ch2);

        if (ch1 >= 2 && (lbr->spatial_info_pres & (1 << ch1)) && 0) {
            decode_spatial_info(lbr, 0, ch1);
            if (ch1 != ch2 && (lbr->spatial_info_pres & (1 << ch2)))
                decode_spatial_info(lbr, 1, ch2);
        }

        transform_channel(lbr, ch1);
        if (ch1 != ch2)
            transform_channel(lbr, ch2);
    }

    int ch = 0;
    lbr->output_mask = 0;
    if (lbr->ch_mask & SPEAKER_PAIR_LR) {
        lbr->output_samples[SPEAKER_L] = lbr->channel_buffer[ch++];
        lbr->output_samples[SPEAKER_R] = lbr->channel_buffer[ch++];
        lbr->output_mask |= SPEAKER_MASK_L | SPEAKER_MASK_R;
    }
    if (lbr->ch_mask & SPEAKER_PAIR_LsRs) {
        lbr->output_samples[SPEAKER_Ls] = lbr->channel_buffer[ch++];
        lbr->output_samples[SPEAKER_Rs] = lbr->channel_buffer[ch++];
        lbr->output_mask |= SPEAKER_MASK_Ls | SPEAKER_MASK_Rs;
    }
    if (lbr->ch_mask & SPEAKER_PAIR_C) {
        lbr->output_samples[SPEAKER_C] = lbr->channel_buffer[ch++];
        lbr->output_mask |= SPEAKER_MASK_C;
    }
    if (lbr->flags & LBR_FLAG_LFE_PRESENT) {
        lbr->output_samples[SPEAKER_LFE1] = lbr->channel_buffer[ch++];
        lbr->output_mask |= SPEAKER_MASK_LFE1;
        interpolate_lfe(lbr);
    }

    return 0;
}

void lbr_clear(struct lbr_decoder *lbr)
{
    if (lbr) {
        memset(lbr->part_stereo, 16, sizeof(lbr->part_stereo));
        memset(lbr->spatial_info, 16, sizeof(lbr->spatial_info));
        memset(lbr->lpc_coeff, 0, sizeof(lbr->lpc_coeff));
        memset(lbr->time_samples, 0, sizeof(lbr->time_samples));
        memset(lbr->imdct_history, 0, sizeof(lbr->imdct_history));
        memset(lbr->tonal_bounds, 0, sizeof(lbr->tonal_bounds));
        memset(lbr->lfe_history, 0, sizeof(lbr->lfe_history));
        lbr->framenum = 0;
        lbr->ntones = 0;
    }
}

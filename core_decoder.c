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
#include "interpolator.h"
#include "fixed_math.h"
#include "core_decoder.h"
#include "exss_parser.h"
#include "dmix_tables.h"

#include "core_tables.h"
#include "core_huffman.h"
#include "core_vectors.h"

enum sample_type {
    NO_BITS_ALLOCATED,
    HUFFMAN_CODE,
    BLOCK_CODE,
    NO_FURTHER_ENCODING
};

enum header_type {
    HEADER_CORE,
    HEADER_XCH,
    HEADER_XXCH
};

// Mode 0: A (mono)
// Mode 1: A + B (dual mono)
// Mode 2: L + R (stereo)
// Mode 3: (L+R) + (L-R) (sum-diff)
// Mode 4: LT + RT (left and right total)
// Mode 5: C + L + R
// Mode 6: L + R + S
// Mode 7: C + L + R + S
// Mode 8: L + R + SL + SR
// Mode 9: C + L + R + SL + SR

static const int8_t prm_ch_to_spkr_map[10][5] = {
    { SPEAKER_C,        -1,         -1,         -1,         -1 },
    { SPEAKER_L, SPEAKER_R,         -1,         -1,         -1 },
    { SPEAKER_L, SPEAKER_R,         -1,         -1,         -1 },
    { SPEAKER_L, SPEAKER_R,         -1,         -1,         -1 },
    { SPEAKER_L, SPEAKER_R,         -1,         -1,         -1 },
    { SPEAKER_C, SPEAKER_L, SPEAKER_R ,         -1,         -1 },
    { SPEAKER_L, SPEAKER_R, SPEAKER_Cs,         -1          -1 },
    { SPEAKER_C, SPEAKER_L, SPEAKER_R , SPEAKER_Cs,         -1 },
    { SPEAKER_L, SPEAKER_R, SPEAKER_Ls, SPEAKER_Rs,         -1 },
    { SPEAKER_C, SPEAKER_L, SPEAKER_R,  SPEAKER_Ls, SPEAKER_Rs }
};

static const uint8_t audio_mode_ch_mask[10] = {
    SPEAKER_MASK_C,
    SPEAKER_MASK_L | SPEAKER_MASK_R,
    SPEAKER_MASK_L | SPEAKER_MASK_R,
    SPEAKER_MASK_L | SPEAKER_MASK_R,
    SPEAKER_MASK_L | SPEAKER_MASK_R,
    SPEAKER_MASK_C | SPEAKER_MASK_L | SPEAKER_MASK_R,
    SPEAKER_MASK_L | SPEAKER_MASK_R | SPEAKER_MASK_Cs,
    SPEAKER_MASK_C | SPEAKER_MASK_L | SPEAKER_MASK_R  | SPEAKER_MASK_Cs,
    SPEAKER_MASK_L | SPEAKER_MASK_R | SPEAKER_MASK_Ls | SPEAKER_MASK_Rs,
    SPEAKER_MASK_C | SPEAKER_MASK_L | SPEAKER_MASK_R  | SPEAKER_MASK_Ls | SPEAKER_MASK_Rs
};

// 5.3.1 - Bit stream header
static int parse_frame_header(struct core_decoder *core)
{
    // Frame type
    core->normal_frame = bits_get1(&core->bits);

    // Deficit sample count
    core->deficit_samples = bits_get(&core->bits, 5) + 1;
    enforce(core->deficit_samples == 32 || core->normal_frame == false,
            "Invalid deficit sample count");

    // CRC present flag
    core->crc_present = bits_get1(&core->bits);

    // Number of PCM sample blocks
    core->npcmblocks = bits_get(&core->bits, 7) + 1;
    enforce(core->npcmblocks >= 8, "Invalid number of PCM sample blocks");

    // Primary frame byte size
    core->frame_size = bits_get(&core->bits, 14) + 1;
    enforce(core->frame_size >= 96, "Invalid frame size");

    // Audio channel arrangement
    core->audio_mode = bits_get(&core->bits, 6);
    require(core->audio_mode < 10, "Unsupported audio channel arrangement");

    // Core audio sampling frequency
    core->sample_rate = sample_rates[bits_get(&core->bits, 4)];
    enforce(core->sample_rate != 0, "Invalid core audio sampling frequency");

    // Transmission bit rate
    core->bit_rate = bit_rates[bits_get(&core->bits, 5)];
    enforce(core->bit_rate != -1, "Invalid transmission bit rate");

    // Reserved field
    bits_skip1(&core->bits);

    // Embedded dynamic range flag
    core->drc_present = bits_get1(&core->bits);

    // Embedded time stamp flag
    core->ts_present = bits_get1(&core->bits);

    // Auxiliary data flag
    core->aux_present = bits_get1(&core->bits);

    // HDCD mastering flag
    bits_skip1(&core->bits);

    // Extension audio descriptor flag
    // 0 - Channel extension (XCH)
    // 2 - Frequency extension (X96)
    // 6 - Channel extension (XXCH)
    core->ext_audio_type = bits_get(&core->bits, 3);

    // Extended coding flag
    core->ext_audio_present = bits_get1(&core->bits);

    // Audio sync word insertion flag
    core->sync_ssf = bits_get1(&core->bits);

    // Low frequency effects flag
    core->lfe_present = bits_get(&core->bits, 2);
    enforce(core->lfe_present < 3, "Invalid low frequency effects flag");

    // Predictor history flag switch
    core->predictor_history = bits_get1(&core->bits);

    // Header CRC check bytes
    if (core->crc_present)
        bits_skip(&core->bits, 16);

    // Multirate interpolator switch
    core->filter_perfect = bits_get1(&core->bits);

    // Encoder software revision
    bits_skip(&core->bits, 4);

    // Copy history
    bits_skip(&core->bits, 2);

    // Source PCM resolution
    int pcmr_index = bits_get(&core->bits, 3);
    core->source_pcm_res = sample_res[pcmr_index];
    enforce(core->source_pcm_res != 0, "Invalid source PCM resolution");
    core->es_format = !!(pcmr_index & 1);

    // Front sum/difference flag
    require(bits_get1(&core->bits) == false, "Front sum/difference not supported");

    // Surround sum/difference flag
    require(bits_get1(&core->bits) == false, "Surround sum/difference not supported");

    // Dialog normalization / unspecified
    bits_skip(&core->bits, 4);

    return 0;
}

// 5.3.2 - Primary audio coding header
static int parse_coding_header(struct core_decoder *core, enum header_type header, int xch_base)
{
    int ch, n, ret;

    size_t header_pos = core->bits.index;
    size_t header_size = 0;

    switch (header) {
    case HEADER_CORE:
        // Number of subframes
        core->nsubframes = bits_get(&core->bits, 4) + 1;

        // Number of primary audio channels
        core->nchannels = bits_get(&core->bits, 3) + 1;
        enforce(core->nchannels == audio_mode_nch[core->audio_mode],
                "Invalid number of primary audio channels");
        assert(core->nchannels <= MAX_CHANNELS - 2);

        core->ch_mask = audio_mode_ch_mask[core->audio_mode];

        core->dmix_coeffs_present = core->dmix_embedded = false;
        break;

    case HEADER_XCH:
        // Number of extension channels
        n = bits_get(&core->bits, 3) + 1;
        require(n == 1, "Too many XCH audio channels");
        core->nchannels += n;
        assert(core->nchannels <= MAX_CHANNELS - 1);
        core->ch_mask |= SPEAKER_MASK_Cs;
        break;

    case HEADER_XXCH:
        // Channel set header length
        header_size = bits_get(&core->bits, 7) + 1;

        // Check CRC
        if (core->xxch_crc_present)
            if ((ret = bits_check_crc(&core->bits, header_pos, header_pos + header_size * 8)) < 0)
                return ret;

        // Number of channels in a channel set
        n = bits_get(&core->bits, 3) + 1;
        require(n < 3, "Too many XXCH audio channels");
        core->nchannels += n;
        assert(core->nchannels <= MAX_CHANNELS);

        // Loudspeaker activity mask
        core->ch_mask |= bits_get(&core->bits, core->xxch_mask_nbits - 6) << 6;

        // Downmix coefficients present in stream
        core->dmix_coeffs_present = bits_get1(&core->bits);

        if (core->dmix_coeffs_present) {
            // Downmix already performed by encoder
            core->dmix_embedded = bits_get1(&core->bits);

            // Downmix scale factor
            core->dmix_scale = bits_get(&core->bits, 6);

            // Downmix channel mapping mask
            for (ch = xch_base; ch < core->nchannels; ch++)
                core->dmix_mask[ch] = bits_get(&core->bits, core->xxch_mask_nbits);

            // Downmix coefficients
            int *coeff_ptr = core->dmix_coeff;
            for (ch = xch_base; ch < core->nchannels; ch++)
                for (n = 0; n < core->xxch_mask_nbits; n++)
                    if (core->dmix_mask[ch] & (1 << n))
                        *coeff_ptr++ = bits_get(&core->bits, 7);
        } else {
            core->dmix_embedded = false;
        }

        /*
        printf("nchannels %d\n", core->nchannels);
        printf("ch_mask %#x\n", core->ch_mask);
        printf("dmix_coeffs_present %d\n", core->dmix_coeffs_present);
        if (core->dmix_coeffs_present) {
            printf("dmix_embedded %d\n", core->dmix_embedded);
            printf("dmix_scale %d\n", core->dmix_scale);
            for (ch = xch_base; ch < core->nchannels; ch++)
                printf("dmix_mask[%d] %#x\n", ch, core->dmix_mask[ch]);
        }
        */
        break;
    }

    // Subband activity count
    for (ch = xch_base; ch < core->nchannels; ch++)
        core->nsubbands[ch] = bits_get(&core->bits, 5) + 2;

    // High frequency VQ start subband
    for (ch = xch_base; ch < core->nchannels; ch++)
        core->subband_vq_start[ch] = bits_get(&core->bits, 5) + 1;

    // Joint intensity coding index
    for (ch = xch_base; ch < core->nchannels; ch++) {
        core->joint_intensity_index[ch] = bits_get(&core->bits, 3);
        require(core->joint_intensity_index[ch] == 0, "Joint subband coding not supported");
    }

    // Transient mode code book
    for (ch = xch_base; ch < core->nchannels; ch++)
        core->transition_mode_sel[ch] = bits_get(&core->bits, 2);

    // Scale factor code book
    for (ch = xch_base; ch < core->nchannels; ch++) {
        core->scale_factor_sel[ch] = bits_get(&core->bits, 3);
        enforce(core->scale_factor_sel[ch] < 7, "Invalid scale factor code book");
    }

    // Bit allocation quantizer select
    for (ch = xch_base; ch < core->nchannels; ch++) {
        core->bit_allocation_sel[ch] = bits_get(&core->bits, 3);
        enforce(core->bit_allocation_sel[ch] < 7, "Invalid bit allocation quantizer select");
    }

    // Quantization index codebook select
    n = 0;  // ABITS = 1
    for (ch = xch_base; ch < core->nchannels; ch++)
        core->quant_index_sel[ch][n] = bits_get(&core->bits, 1);

    for (n = 1; n < 5; n++) // ABITS = 2 to 5
        for (ch = xch_base; ch < core->nchannels; ch++)
            core->quant_index_sel[ch][n] = bits_get(&core->bits, 2);

    for (n = 5; n < NUM_CODE_BOOKS; n++)    // ABITS = 6 to 10
        for (ch = xch_base; ch < core->nchannels; ch++)
            core->quant_index_sel[ch][n] = bits_get(&core->bits, 3);

    // Scale factor adjustment index
    n = 0;  // ABITS = 1
    for (ch = xch_base; ch < core->nchannels; ch++)
        if (core->quant_index_sel[ch][n] == 0)
            core->scale_factor_adj[ch][n] = scale_factor_adj[bits_get(&core->bits, 2)];

    for (n = 1; n < 5; n++) // ABITS = 2 to 5
        for (ch = xch_base; ch < core->nchannels; ch++)
            if (core->quant_index_sel[ch][n] < 3)
                core->scale_factor_adj[ch][n] = scale_factor_adj[bits_get(&core->bits, 2)];

    for (n = 5; n < NUM_CODE_BOOKS; n++)    // ABITS = 6 to 10
        for (ch = xch_base; ch < core->nchannels; ch++)
            if (core->quant_index_sel[ch][n] < 7)
                core->scale_factor_adj[ch][n] = scale_factor_adj[bits_get(&core->bits, 2)];

    if (header == HEADER_XXCH) {
        // Reserved
        // Byte align
        // CRC16 of channel set header
        if ((ret = bits_seek(&core->bits, header_pos + header_size * 8)) < 0)
            return ret;
    } else {
        // Audio header CRC check word
        if (core->crc_present)
            bits_skip(&core->bits, 16);
    }

    return 0;
}

static int parse_scale(struct core_decoder *core, int *scale_index, int sel)
{
    // Select the root square table
    const int32_t *scale_table;
    size_t scale_size;
    if (sel > 5) {
        scale_table = scale_factors_7bit;
        scale_size = dca_countof(scale_factors_7bit);
    } else {
        scale_table = scale_factors_6bit;
        scale_size = dca_countof(scale_factors_6bit);
    }

    if (sel < 5)
        // If Huffman code was used, the difference of scales was encoded
        *scale_index += bits_get_signed_vlc(&core->bits, &scale_factor_huff[sel]);
    else
        *scale_index = bits_get(&core->bits, sel + 1);

    // Look up scale factor from the root square table
    enforce((unsigned int)*scale_index < scale_size, "Invalid scale factor index");
    return scale_table[*scale_index];
}

// 5.4.1 - Primary audio coding side information
static int parse_subframe_header(struct core_decoder *core, int sf,
                                 enum header_type header, int xch_base)
{
    int ch, band;

    if (header == HEADER_CORE) {
        // Subsubframe count
        core->nsubsubframes[sf] = bits_get(&core->bits, 2) + 1;

        // Partial subsubframe sample count
        bits_skip(&core->bits, 3);
    }

    // Prediction mode
    for (ch = xch_base; ch < core->nchannels; ch++)
        for (band = 0; band < core->nsubbands[ch]; band++)
            core->prediction_mode[ch][band] = bits_get1(&core->bits);

    // Prediction coefficients VQ address
    for (ch = xch_base; ch < core->nchannels; ch++)
        for (band = 0; band < core->nsubbands[ch]; band++)
            if (core->prediction_mode[ch][band])
                core->prediction_vq_index[ch][band] = bits_get(&core->bits, 12);

    // Bit allocation index
    for (ch = xch_base; ch < core->nchannels; ch++) {
        // Not high frequency VQ subbands
        for (band = 0; band < core->subband_vq_start[ch]; band++) {
            // Select codebook
            int abits, sel = core->bit_allocation_sel[ch];
            if (sel < 5)
                abits = bits_get_unsigned_vlc(&core->bits, &bit_allocation_huff[sel]) + 1;
            else
                abits = bits_get(&core->bits, sel - 1);
            enforce(abits < 27, "Invalid bit allocation index");
            core->bit_allocation[ch][band] = abits;
        }
    }

    // Transition mode
    for (ch = xch_base; ch < core->nchannels; ch++) {
        // Clear transition mode for all subbands
        for (band = 0; band < core->nsubbands[ch]; band++)
            core->transition_mode[sf][ch][band] = 0;

        // Transient possible only if more than one subsubframe
        if (core->nsubsubframes[sf] > 1) {
            // Not high frequency VQ subbands
            for (band = 0; band < core->subband_vq_start[ch]; band++) {
                // Present only if bits allocated
                if (core->bit_allocation[ch][band]) {
                    int sel = core->transition_mode_sel[ch];
                    const struct huffman *huff = &transition_mode_huff[sel];
                    int trans_ssf = bits_get_unsigned_vlc(&core->bits, huff);
                    enforce(trans_ssf < 4, "Invalid transition mode index");
                    core->transition_mode[sf][ch][band] = trans_ssf;
                }
            }
        }
    }

    // Scale factors
    for (ch = xch_base; ch < core->nchannels; ch++) {
        int ret;

        // Clear scale factors
        for (band = 0; band < core->nsubbands[ch]; band++) {
            core->scale_factors[ch][band][0] = 0;
            core->scale_factors[ch][band][1] = 0;
        }

        // Select codebook
        int sel = core->scale_factor_sel[ch];

        // Clear accumulation
        int scale_index = 0;

        // Extract scales for subbands up to VQ
        for (band = 0; band < core->subband_vq_start[ch]; band++) {
            if (core->bit_allocation[ch][band]) {
                if ((ret = parse_scale(core, &scale_index, sel)) < 0)
                    return ret;
                core->scale_factors[ch][band][0] = ret;
                if (core->transition_mode[sf][ch][band]) {
                    if ((ret = parse_scale(core, &scale_index, sel)) < 0)
                        return ret;
                    core->scale_factors[ch][band][1] = ret;
                }
            }
        }

        // High frequency VQ subbands
        for (band = core->subband_vq_start[ch]; band < core->nsubbands[ch]; band++) {
            if ((ret = parse_scale(core, &scale_index, sel)) < 0)
                return ret;
            core->scale_factors[ch][band][0] = ret;
        }
    }

    // Joint subband codebook select
    for (ch = xch_base; ch < core->nchannels; ch++) {
        // Only if joint subband coding is enabled
        if (core->joint_intensity_index[ch]) {
            core->joint_scale_sel[ch] = bits_get(&core->bits, 3);
            enforce(core->joint_scale_sel[ch] < 7, "Invalid joint scale factor code book");
        }
    }

    // Scale factors for joint subband coding
    for (ch = xch_base; ch < core->nchannels; ch++) {
        // Only if joint subband coding is enabled
        if (core->joint_intensity_index[ch]) {
            // Select codebook
            int sel = core->joint_scale_sel[ch];
            // Get source channel
            int src_ch = core->joint_intensity_index[ch] - 1;
            for (band = core->nsubbands[ch]; band < core->nsubbands[src_ch]; band++) {
                int scale_index;

                if (sel < 5)
                    scale_index = bits_get_signed_vlc(&core->bits, &scale_factor_huff[sel]);
                else
                    scale_index = bits_get(&core->bits, sel + 1);

                // Bias by 64
                scale_index += 64;

                enforce((unsigned int)scale_index < dca_countof(joint_scale_factors),
                        "Invalid joint scale factor index");
                core->joint_scale_factors[ch][band] = joint_scale_factors[scale_index];
            }
        }
    }

    // Dynamic range coefficient
    if (core->drc_present && header == HEADER_CORE)
        bits_skip(&core->bits, 8);

    // Side information CRC check word
    if (core->crc_present)
        bits_skip(&core->bits, 16);

    return 0;
}

static int parse_block_code(struct core_decoder *core, int *value, int sel)
{
    // Select block code book
    // Extract the block code index from the bit stream
    int code = bits_get(&core->bits, block_code_bits[sel]);
    int levels = quant_levels[sel];
    int offset = (levels - 1) >> 1;

    // Look up 4 samples from the block code book
    for (int n = 0; n < 4; n++) {
        value[n] = (code % levels) - offset;
        code /= levels;
    }

    enforce(code == 0, "Failed to decode block code");
    return 0;
}

static inline void dequantize(int *output, const int *input, int step_size,
                              int scale, bool residual)
{
    // Account for quantizer step size
    int64_t step_scale = (int64_t)step_size * scale;
    int nbits = 64 - dca_clz64(step_scale | INT64_C(1));
    int shift = nbits > 23 ? nbits - 23 : 0;
    int32_t _step_scale = (int32_t)(step_scale >> shift);

    // Scale the samples
    if (residual) {
        for (int n = 0; n < NUM_SUBBAND_SAMPLES; n++)
            output[n] += clip23(mul__(input[n], _step_scale, 22 - shift));
    } else {
        for (int n = 0; n < NUM_SUBBAND_SAMPLES; n++)
            output[n] = clip23(mul__(input[n], _step_scale, 22 - shift));
    }
}

// 5.5 - Primary audio data arrays
static int parse_subband_samples(struct core_decoder *core, int sf, int ssf,
                                 int ch, int band, int sub_pos)
{
    const struct huffman *huff = NULL;

    // Assume no further encoding by default
    enum sample_type type = NO_FURTHER_ENCODING;
    // Select the quantizer
    int abits = core->bit_allocation[ch][band];
    if (abits == 0) {
        // No bits allocated
        type = NO_BITS_ALLOCATED;
    } else if (abits <= NUM_CODE_BOOKS) {
        // Select the group of code books
        const struct huffman *group_huff = quant_index_group_huff[abits - 1];
        int group_size = quant_index_group_size[abits - 1];
        // Select quantization index code book
        int sel = core->quant_index_sel[ch][abits - 1];
        if (sel < group_size) {
            type = HUFFMAN_CODE;
            huff = &group_huff[sel];
        } else if (abits <= 7) {
            type = BLOCK_CODE;
        }
    }

    // Extract bits from the bit stream
    int audio[NUM_SUBBAND_SAMPLES], ret;
    switch (type) {
    case NO_BITS_ALLOCATED:
        memset(audio, 0, sizeof(audio));
        break;
    case HUFFMAN_CODE:
        if ((ret = bits_get_signed_vlc_array(&core->bits, audio, NUM_SUBBAND_SAMPLES, huff)) < 0)
            return ret;
        break;
    case BLOCK_CODE:
        if ((ret = parse_block_code(core, audio + 0, abits)) < 0)
            return ret;
        if ((ret = parse_block_code(core, audio + 4, abits)) < 0)
            return ret;
        break;
    case NO_FURTHER_ENCODING:
        bits_get_signed_array(&core->bits, audio, NUM_SUBBAND_SAMPLES, abits - 3);
        break;
    }

    int step_size, trans_ssf, scale;

    // Select quantization step size table
    // Look up quantization step size
    if (core->bit_rate == -2)
        step_size = step_size_lossless[abits];
    else
        step_size = step_size_lossy[abits];

    // Identify transient location
    trans_ssf = core->transition_mode[sf][ch][band];

    // Determine proper scale factor
    if (trans_ssf == 0 || ssf < trans_ssf)
        scale = core->scale_factors[ch][band][0];
    else
        scale = core->scale_factors[ch][band][1];

    // Adjustment of scale factor
    // Only when SEL indicates Huffman code
    if (type == HUFFMAN_CODE)
        scale = clip23(mul22nrd(core->scale_factor_adj[ch][abits - 1], scale));

    dequantize(core->subband_samples[ch][band] +
               sub_pos + ssf * NUM_SUBBAND_SAMPLES,
               audio, step_size, scale, false);
    return 0;
}

// 5.5 - Primary audio data arrays
static int parse_subframe_audio(struct core_decoder *core, int sf, enum header_type header,
                                int xch_base, int *sub_pos, int *lfe_pos)
{
    int ssf, ch, band;

    // Number of subband samples in this subframe
    int nsamples = core->nsubsubframes[sf] * NUM_SUBBAND_SAMPLES;
    enforce(*sub_pos + nsamples <= core->npcmblocks, "Subband sample buffer overflow");

    // VQ encoded subbands
    for (ch = xch_base; ch < core->nchannels; ch++) {
        for (band = core->subband_vq_start[ch]; band < core->nsubbands[ch]; band++) {
            // Extract the VQ address from the bit stream
            int vq_index = bits_get(&core->bits, 10);

            // Get the scale factor
            int scale = core->scale_factors[ch][band][0];

            // Look up the VQ code book for 32 subband samples
            const int8_t *vq_samples = high_freq_samples[vq_index];

            // Scale and take the samples
            int *samples = core->subband_samples[ch][band] + *sub_pos;
            for (int n = 0; n < nsamples; n++)
                samples[n] = clip23(mul4(scale, vq_samples[n]));
        }
    }

    // Low frequency effect data
    if (core->lfe_present && header == HEADER_CORE) {
        // Number of LFE samples in this subframe
        int nlfesamples = 2 * core->lfe_present * core->nsubsubframes[sf];
        assert(nlfesamples <= MAX_LFE_SAMPLES);

        // Extract LFE samples from the bit stream
        int audio[MAX_LFE_SAMPLES];
        bits_get_signed_array(&core->bits, audio, nlfesamples, 8);

        // Extract scale factor index from the bit stream
        unsigned int scale_index = bits_get(&core->bits, 8);
        enforce(scale_index < dca_countof(scale_factors_7bit),
                "Invalid LFE scale factor index");

        // Look up the 7-bit root square quantization table
        int scale = scale_factors_7bit[scale_index];

        // Account for quantizer step size which is 0.035
        int step_scale = mul23(4697620, scale);

        // Scale the LFE samples
        int *samples = core->lfe_samples + *lfe_pos;
        for (int n = 0; n < nlfesamples; n++)
            samples[n] = clip23((audio[n] * step_scale) >> 4);

        // Advance LFE sample pointer for the next subframe
        *lfe_pos += nlfesamples;
    }

    // Audio data
    for (ssf = 0; ssf < core->nsubsubframes[sf]; ssf++) {
        int ret;

        for (ch = xch_base; ch < core->nchannels; ch++)
            // Not high frequency VQ subbands
            for (band = 0; band < core->subband_vq_start[ch]; band++)
                if ((ret = parse_subband_samples(core, sf, ssf, ch, band, *sub_pos)) < 0)
                    return ret;

        // DSYNC
        if (ssf == core->nsubsubframes[sf] - 1 || core->sync_ssf)
            enforce(bits_get(&core->bits, 16) == 0xffff, "DSYNC check failed");
    }

    // Inverse ADPCM
    for (ch = xch_base; ch < core->nchannels; ch++) {
        for (band = 0; band < core->nsubbands[ch]; band++) {
            // Only if prediction mode is on
            if (core->prediction_mode[ch][band]) {
                int *samples = core->subband_samples[ch][band] + *sub_pos;

                // Extract the VQ index
                int vq_index = core->prediction_vq_index[ch][band];

                // Look up the VQ table for prediction coefficients
                const int16_t *vq_coeffs = adpcm_coeffs[vq_index];
                for (int m = 0; m < nsamples; m++) {
                    int64_t err = INT64_C(0);
                    for (int n = 0; n < NUM_ADPCM_COEFFS; n++)
                        err += (int64_t)samples[m - n - 1] * vq_coeffs[n];
                    samples[m] = clip23(samples[m] + clip23(norm13(err)));
                }
            }
        }
    }

    // Joint subband coding
    for (ch = xch_base; ch < core->nchannels; ch++) {
        // Only if joint subband coding is enabled
        if (core->joint_intensity_index[ch]) {
            // Get source channel
            int src_ch = core->joint_intensity_index[ch] - 1;
            for (band = core->nsubbands[ch]; band < core->nsubbands[src_ch]; band++) {
                int *src = core->subband_samples[src_ch][band] + *sub_pos;
                int *dst = core->subband_samples[    ch][band] + *sub_pos;
                int scale = core->joint_scale_factors[ch][band];
                for (int n = 0; n < nsamples; n++)
                    dst[n] = clip23(mul17(src[n], scale));
            }
        }
    }

    // Advance subband sample pointer for the next subframe
    *sub_pos += nsamples;
    return 0;
}

static void erase_adpcm_history(struct core_decoder *core)
{
    // Erase ADPCM history from previous frame if
    // predictor history switch was disabled
    for (int ch = 0; ch < MAX_CHANNELS; ch++) {
        for (int band = 0; band < MAX_SUBBANDS; band++) {
            int *samples = core->subband_samples[ch][band] - NUM_ADPCM_COEFFS;
            for (int n = 0; n < NUM_ADPCM_COEFFS; n++)
                samples[n] = 0;
        }
    }
}

static void update_adpcm_history(struct core_decoder *core, int xch_base)
{
    // Update history for ADPCM
    for (int ch = xch_base; ch < core->nchannels; ch++) {
        for (int band = 0; band < core->nsubbands[ch]; band++) {
            int *samples = core->subband_samples[ch][band] - NUM_ADPCM_COEFFS;
            for (int n = NUM_ADPCM_COEFFS - 1; n >= 0; n--)
                samples[n] = samples[core->npcmblocks + n];
        }
    }
}

static int alloc_sample_buffer(struct core_decoder *core)
{
    int nchsamples = NUM_ADPCM_COEFFS + core->npcmblocks;
    int nframesamples = nchsamples * MAX_CHANNELS * MAX_SUBBANDS;
    int nlfesamples = MAX_LFE_HISTORY + core->npcmblocks / 2;

    // Reallocate subband sample buffer
    int ret;
    if ((ret = dca_realloc(core, &core->subband_buffer, nframesamples + nlfesamples, sizeof(int))) < 0)
        return ret;
    if (ret > 0) {
        for (int ch = 0; ch < MAX_CHANNELS; ch++)
            for (int band = 0; band < MAX_SUBBANDS; band++)
                core->subband_samples[ch][band] = core->subband_buffer +
                    (ch * MAX_SUBBANDS + band) * nchsamples + NUM_ADPCM_COEFFS;
        core->lfe_samples = core->subband_buffer + nframesamples;
    }

    if (!core->predictor_history)
        erase_adpcm_history(core);

    return 0;
}

static int parse_frame_data(struct core_decoder *core, enum header_type header, int xch_base)
{
    int ret;
    if ((ret = parse_coding_header(core, header, xch_base)) < 0)
        return ret;

    int sub_pos = 0;
    int lfe_pos = MAX_LFE_HISTORY;
    for (int sf = 0; sf < core->nsubframes; sf++) {
        int ret;
        if ((ret = parse_subframe_header(core, sf, header, xch_base)) < 0)
            return ret;
        if ((ret = parse_subframe_audio(core, sf, header, xch_base, &sub_pos, &lfe_pos)) < 0)
            return ret;
    }

    update_adpcm_history(core, xch_base);
    return 0;
}

static int map_prm_ch_to_spkr(struct core_decoder *core, int ch)
{
    int pos = audio_mode_nch[core->audio_mode];
    if (ch < pos)
        return prm_ch_to_spkr_map[core->audio_mode][ch];

    for (int spkr = SPEAKER_Cs; spkr < SPEAKER_COUNT; spkr++)
        if (core->ch_mask & (1 << spkr))
            if (pos++ == ch)
                return spkr;

    return -1;
}

static int map_spkr_to_core_spkr(struct core_decoder *core, int spkr)
{
    if (core->ch_mask & (1 << spkr))
        return spkr;
    if (spkr == SPEAKER_Lss && (core->ch_mask & SPEAKER_MASK_Ls))
        return SPEAKER_Ls;
    if (spkr == SPEAKER_Rss && (core->ch_mask & SPEAKER_MASK_Rs))
        return SPEAKER_Rs;
    return -1;
}

static int conv_dmix_scale(int code)
{
    unsigned int index = (code & 63) * 4 - 4;
    if (index < dca_countof(dmix_table)) {
        int sign = (code >> 6) - 1;
        int coeff = dmix_table[index];
        return (coeff ^ sign) - sign;
    }
    return 0;
}

static int conv_dmix_scale_inv(int code)
{
    unsigned int index = code * 4 - 44;
    if (index < dca_countof(dmix_table_inv))
        return dmix_table_inv[index];
    return 0;
}

int core_filter(struct core_decoder *core, int flags)
{
    bool synth_x96 = !!(flags & DCADEC_FLAG_CORE_SYNTH_X96);

    // Output sample rate
    core->output_rate = core->sample_rate << synth_x96;

    // Number of PCM samples in this frame
    core->npcmsamples = (core->npcmblocks * NUM_PCMBLOCK_SAMPLES) << synth_x96;

    // Add LFE channel if present
    if (core->lfe_present)
        core->ch_mask |= SPEAKER_MASK_LFE1;

    // Reallocate PCM output buffer
    int ret;
    if ((ret = dca_realloc(core, &core->output_buffer, core->npcmsamples * dca_popcount(core->ch_mask), sizeof(int))) < 0)
        return ret;

    int *ptr = core->output_buffer;
    for (int spkr = 0; spkr < SPEAKER_COUNT; spkr++) {
        if (core->ch_mask & (1 << spkr)) {
            core->output_samples[spkr] = ptr;
            ptr += core->npcmsamples;
        } else {
            core->output_samples[spkr] = NULL;
        }
    }

    // Filter primary channels
    for (int ch = 0; ch < core->nchannels; ch++) {
        int nsubbands = core->nsubbands[ch];
        if (core->joint_intensity_index[ch])
            nsubbands = core->nsubbands[core->joint_intensity_index[ch] - 1];

        // Allocate subband DSP
        if (!core->subband_dsp[ch])
            if (!(core->subband_dsp[ch] = interpolator_create(core, flags)))
                return -DCADEC_ENOMEM;

        // Map this primary channel to speaker
        int spkr = map_prm_ch_to_spkr(core, ch);
        if (spkr < 0)
            return -DCADEC_EINVAL;

        // Filter bank reconstruction
        core->subband_dsp[ch]->interpolate(core->subband_dsp[ch],
                                           core->output_samples[spkr],
                                           core->subband_samples[ch],
                                           nsubbands,
                                           core->npcmblocks,
                                           core->filter_perfect);
    }

    // Filter LFE channel
    if (core->lfe_present) {
        // Select LFE DSP
        interpolate_lfe_t interpolate;
        if (flags & DCADEC_FLAG_CORE_BIT_EXACT)
            interpolate = interpolate_lfe_fixed_fir;
        else if (flags & DCADEC_FLAG_CORE_LFE_FIR)
            interpolate = interpolate_lfe_float_fir;
        else
            interpolate = interpolate_lfe_float_iir;

        // Interpolation of LFE channel
        interpolate(core->output_samples[SPEAKER_LFE1],
                    core->lfe_samples,
                    core->npcmblocks >> (3 - core->lfe_present),
                    core->lfe_present == 1,
                    synth_x96);

        if (flags & DCADEC_FLAG_CORE_SYNTH_X96) {
            // Filter 96 kHz oversampled LFE PCM to attenuate high frequency
            // (47.6 - 48.0 kHz) components of interpolation image
            int history = core->output_history_lfe;
            int *samples = core->output_samples[SPEAKER_LFE1];
            for (int n = 0; n < core->npcmsamples; n += 2) {
                int64_t res1 = INT64_C(2097471) * samples[n] + INT64_C(6291137) * history;
                int64_t res2 = INT64_C(6291137) * samples[n] + INT64_C(2097471) * history;
                samples[n    ]           = clip23(norm23(res1));
                samples[n + 1] = history = clip23(norm23(res2));
            }

            // Update LFE PCM history
            core->output_history_lfe = history;
        }
    }

    if (!(flags & DCADEC_FLAG_KEEP_DMIX_6CH)) {
        // Undo embedded XCH downmix
        if (core->es_format && core->xch_present && core->audio_mode >= 8) {
            int *samples_ls = core->output_samples[SPEAKER_Ls];
            int *samples_rs = core->output_samples[SPEAKER_Rs];
            int *samples_cs = core->output_samples[SPEAKER_Cs];
            for (int n = 0; n < core->npcmsamples; n++) {
                int cs = mul23(samples_cs[n], 5931520);
                samples_ls[n] = clip23(samples_ls[n] - cs);
                samples_rs[n] = clip23(samples_rs[n] - cs);
            }
        }

        // Undo embedded XXCH downmix
        if (core->dmix_embedded) {
            int scale_inv = conv_dmix_scale_inv(core->dmix_scale);

            // Undo embedded core downmix pre-scaling
            for (int spkr = 0; spkr < SPEAKER_Cs; spkr++) {
                if ((core->ch_mask & (1 << spkr))) {
                    int *samples = core->output_samples[spkr];
                    for (int n = 0; n < core->npcmsamples; n++)
                        samples[n] = mul16(samples[n], scale_inv);
                }
            }

            // Undo downmix
            int *coeff_ptr = core->dmix_coeff;
            for (int ch = audio_mode_nch[core->audio_mode]; ch < core->nchannels; ch++) {
                int spkr1 = map_prm_ch_to_spkr(core, ch);
                if (spkr1 < 0)
                    return -DCADEC_EINVAL;
                for (int spkr2 = 0; spkr2 < core->xxch_mask_nbits; spkr2++) {
                    if (core->dmix_mask[ch] & (1 << spkr2)) {
                        int spkr3 = map_spkr_to_core_spkr(core, spkr2);
                        if (spkr3 < 0)
                            return -DCADEC_EINVAL;
                        int coeff = mul16(conv_dmix_scale(*coeff_ptr++), scale_inv);
                        int *src = core->output_samples[spkr1];
                        int *dst = core->output_samples[spkr3];
                        for (int n = 0; n < core->npcmsamples; n++)
                            dst[n] -= mul15(src[n], coeff);
                    }
                }
            }

            // Clip core channels
            for (int spkr = 0; spkr < SPEAKER_Cs; spkr++) {
                if (core->ch_mask & (1 << spkr)) {
                    int *samples = core->output_samples[spkr];
                    for (int n = 0; n < core->npcmsamples; n++)
                        samples[n] = clip23(samples[n]);
                }
            }
        }
    }

    // Reduce core bit width
    if (flags & DCADEC_FLAG_CORE_SOURCE_PCM_RES) {
        int shift = 24 - core->source_pcm_res;
        if (shift > 0) {
            int round = 1 << (shift - 1);
            for (int spkr = 0; spkr < SPEAKER_COUNT; spkr++) {
                if (core->ch_mask & (1 << spkr)) {
                    int *samples = core->output_samples[spkr];
                    for (int n = 0; n < core->npcmsamples; n++)
                        samples[n] = (samples[n] + round) >> shift;
                }
            }
        }
        core->bits_per_sample = core->source_pcm_res;
    } else {
        core->bits_per_sample = 24;
    }

    return 0;
}

static int parse_xch_frame(struct core_decoder *core)
{
    enforce(!(core->ch_mask & SPEAKER_MASK_Cs), "XCH with Cs speaker already present");

    size_t frame_pos = core->bits.index;

    // XCH frame size
    size_t frame_size = bits_get(&core->bits, 10) + 1;
    enforce(frame_size > 4, "Invalid XCH frame size");

    // Extension channel arrangement
    require(bits_get(&core->bits, 4) == 1, "Unsupported XCH audio mode");

    int ret;
    if ((ret = parse_frame_data(core, HEADER_XCH, core->nchannels)) < 0)
        return ret;

    return bits_seek(&core->bits, frame_pos + frame_size * 8 - 32);
}

static int parse_xxch_frame(struct core_decoder *core)
{
    enforce(!core->xch_present, "XXCH with XCH already present");

    size_t header_pos = core->bits.index;

    // XXCH frame header length
    size_t header_size = bits_get(&core->bits, 6) + 1;
    enforce(header_size > 4, "Invalid XXCH header size");

    size_t header_end = header_pos + header_size * 8 - 32;

    // Check XXCH frame header CRC
    int ret;
    if ((ret = bits_check_crc(&core->bits, header_pos, header_end)) < 0)
        return ret;

    // CRC presence flag for channel set header
    core->xxch_crc_present = bits_get1(&core->bits);

    // Number of bits for loudspeaker mask
    core->xxch_mask_nbits = bits_get(&core->bits, 5) + 1;
    enforce(core->xxch_mask_nbits > 6, "Invalid number of bits for XXCH speaker mask");

    // Number of channel sets
    int xxch_nchsets = bits_get(&core->bits, 2) + 1;
    require(xxch_nchsets == 1, "Unsupported number of XXCH channel sets");

    // Channel set 0 data byte size
    int xxch_frame_size = bits_get(&core->bits, 14) + 1;

    // Core loudspeaker activity mask
    core->xxch_core_mask = bits_get(&core->bits, core->xxch_mask_nbits);

    /*
    printf("xxch_crc_present %d\n", core->xxch_crc_present);
    printf("xxch_mask_nbits %d\n", core->xxch_mask_nbits);
    printf("xxch_frame_size %d\n", xxch_frame_size);
    printf("xxch_core_mask %#x\n", core->xxch_core_mask);
    */

    // Reserved
    // Byte align
    // CRC16 of XXCH frame header
    if ((ret = bits_seek(&core->bits, header_end)) < 0)
        return ret;

    // Parse XXCH channel set 0
    if ((ret = parse_frame_data(core, HEADER_XXCH, core->nchannels)) < 0)
        return ret;

    return bits_seek(&core->bits, header_end + xxch_frame_size * 8);
}

static int parse_xbr_subframe(struct core_decoder *core, int xbr_base_ch, int xbr_nchannels,
                              int *xbr_nsubbands, bool xbr_transition_mode, int sf, int *sub_pos)
{
    int     xbr_nabits[MAX_CHANNELS];
    int     xbr_bit_allocation[MAX_CHANNELS][MAX_SUBBANDS];
    int     xbr_scale_nbits[MAX_CHANNELS];
    int     xbr_scale_factors[MAX_CHANNELS][MAX_SUBBANDS][2];
    int     ch, band, ssf;

    // Number of subband samples in this subframe
    int nsamples = core->nsubsubframes[sf] * NUM_SUBBAND_SAMPLES;
    enforce(*sub_pos + nsamples <= core->npcmblocks, "Subband sample buffer overflow");

    // Number of bits for XBR bit allocation index
    for (ch = xbr_base_ch; ch < xbr_nchannels; ch++)
        xbr_nabits[ch] = bits_get(&core->bits, 2) + 2;

    // XBR bit allocation index
    for (ch = xbr_base_ch; ch < xbr_nchannels; ch++)
        for (band = 0; band < xbr_nsubbands[ch]; band++)
            xbr_bit_allocation[ch][band] = bits_get(&core->bits, xbr_nabits[ch]);

    // Number of bits for scale indices
    for (ch = xbr_base_ch; ch < xbr_nchannels; ch++) {
        xbr_scale_nbits[ch] = bits_get(&core->bits, 3);
        enforce(xbr_scale_nbits[ch] > 0, "Invalid number of bits for XBR scale factor index");
    }

    // XBR scale factors
    for (ch = xbr_base_ch; ch < xbr_nchannels; ch++) {
        // Select the root square table
        const int32_t *scale_table;
        size_t scale_size;
        if (core->scale_factor_sel[ch] > 5) {
            scale_table = scale_factors_7bit;
            scale_size = dca_countof(scale_factors_7bit);
        } else {
            scale_table = scale_factors_6bit;
            scale_size = dca_countof(scale_factors_6bit);
        }

        // Parse scale factor indices
        // Look up scale factors from the root square table
        for (band = 0; band < xbr_nsubbands[ch]; band++) {
            if (xbr_bit_allocation[ch][band] > 0) {
                unsigned int scale_index = bits_get(&core->bits, xbr_scale_nbits[ch]);
                enforce(scale_index < scale_size, "Invalid XBR scale factor index");
                xbr_scale_factors[ch][band][0] = scale_table[scale_index];
                if (xbr_transition_mode && core->transition_mode[sf][ch][band]) {
                    scale_index = bits_get(&core->bits, xbr_scale_nbits[ch]);
                    enforce(scale_index < scale_size, "Invalid XBR scale factor index");
                    xbr_scale_factors[ch][band][1] = scale_table[scale_index];
                }
            }
        }
    }

    // Audio data
    for (ssf = 0; ssf < core->nsubsubframes[sf]; ssf++) {
        for (ch = xbr_base_ch; ch < xbr_nchannels; ch++) {
            for (band = 0; band < xbr_nsubbands[ch]; band++) {
                int audio[NUM_SUBBAND_SAMPLES];

                // Select the quantizer
                int abits = xbr_bit_allocation[ch][band];
                if (abits > 7) {
                    // No further encoding
                    bits_get_signed_array(&core->bits, audio, NUM_SUBBAND_SAMPLES, abits - 3);
                } else if (abits > 0) {
                    // Block codes
                    int ret;
                    if ((ret = parse_block_code(core, audio + 0, abits)) < 0)
                        return ret;
                    if ((ret = parse_block_code(core, audio + 4, abits)) < 0)
                        return ret;
                } else {
                    // No bits allocated
                    continue;
                }

                int step_size, trans_ssf, scale;

                // Look up quantization step size
                step_size = step_size_lossless[abits];

                // Identify transient location
                if (xbr_transition_mode)
                    trans_ssf = core->transition_mode[sf][ch][band];
                else
                    trans_ssf = 0;

                // Determine proper scale factor
                if (trans_ssf == 0 || ssf < trans_ssf)
                    scale = xbr_scale_factors[ch][band][0];
                else
                    scale = xbr_scale_factors[ch][band][1];

                dequantize(core->subband_samples[ch][band] +
                           *sub_pos + ssf * NUM_SUBBAND_SAMPLES,
                           audio, step_size, scale, true);
            }
        }

        // DSYNC
        if (ssf == core->nsubsubframes[sf] - 1 || core->sync_ssf)
            enforce(bits_get(&core->bits, 16) == 0xffff, "DSYNC check failed");
    }

    // Advance subband sample pointer for the next subframe
    *sub_pos += nsamples;
    return 0;
}

static int parse_xbr_frame(struct core_decoder *core)
{
    int     xbr_frame_size[4];
    int     xbr_nchannels[4];
    int     xbr_nsubbands[4 * 8];

    size_t header_pos = core->bits.index;

    // XBR frame header length
    size_t header_size = bits_get(&core->bits, 6) + 1;
    enforce(header_size > 4, "Invalid XBR header size");

    // Check XBR frame header CRC
    int ret;
    if ((ret = bits_check_crc(&core->bits, header_pos, header_pos + header_size * 8 - 32)) < 0)
        return ret;

    // Number of channel sets
    int xbr_nchsets = bits_get(&core->bits, 2) + 1;

    // Channel set data byte size
    for (int i = 0; i < xbr_nchsets; i++)
        xbr_frame_size[i] = bits_get(&core->bits, 14) + 1;

    // Transition mode flag
    bool xbr_transition_mode = bits_get1(&core->bits);

    // Channel set headers
    int xbr_base_ch = 0;
    for (int i = 0; i < xbr_nchsets; i++) {
        xbr_nchannels[i] = bits_get(&core->bits, 3) + 1;
        int xbr_band_nbits = bits_get(&core->bits, 2) + 5;
        for (int ch = 0; ch < xbr_nchannels[i]; ch++)
            xbr_nsubbands[xbr_base_ch++] = bits_get(&core->bits, xbr_band_nbits) + 1;
    }

    /*
    printf("xbr_nchsets %d\n", xbr_nchsets);
    printf("xbr_transition_mode %d\n", xbr_transition_mode);
    for (int i = 0; i < xbr_nchsets; i++)
        printf("xbr_nchannels[%d] %d\n", i, xbr_nchannels[i]);
    */

    // Reserved
    // Byte align
    // CRC16 of XBR frame header
    if ((ret = bits_seek(&core->bits, header_pos + header_size * 8 - 32)) < 0)
        return ret;

    // Channel set data
    xbr_base_ch = 0;
    for (int i = 0; i < xbr_nchsets; i++) {
        header_pos = core->bits.index;

        if (xbr_base_ch + xbr_nchannels[i] <= MAX_CHANNELS) {
            int sub_pos = 0;
            for (int sf = 0; sf < core->nsubframes; sf++)
                if ((ret = parse_xbr_subframe(core, xbr_base_ch,
                                              xbr_base_ch + xbr_nchannels[i],
                                              xbr_nsubbands, xbr_transition_mode,
                                              sf, &sub_pos)) < 0)
                    return ret;
        }

        xbr_base_ch += xbr_nchannels[i];

        if ((ret = bits_seek(&core->bits, header_pos + xbr_frame_size[i] * 8)) < 0)
            return ret;
    }

    return 0;
}

// Revert to base core channel set in case (X)XCH parsing fails
static void revert_to_base_chset(struct core_decoder *core)
{
    core->nchannels = audio_mode_nch[core->audio_mode];
    core->ch_mask = audio_mode_ch_mask[core->audio_mode];
    core->dmix_coeffs_present = core->dmix_embedded = false;
}

static int parse_optional_info(struct core_decoder *core, int flags)
{
    // Only when extensions decoding is requested
    if (!core->ext_audio_present || (flags & DCADEC_FLAG_CORE_ONLY))
        return 0;

    // Time code stamp
    if (core->ts_present)
        bits_skip(&core->bits, 32);

    if (core->aux_present) {
        // Auxiliary data byte count
        size_t aux_size = bits_get(&core->bits, 6);

        // Auxiliary data sync word aligned on 4-byte boundary
        bits_align4(&core->bits);

        // Auxiliary data bytes
        bits_skip(&core->bits, aux_size * 8);
    }

    // Optional CRC check bytes
    if (core->crc_present && core->drc_present) {
        bits_align1(&core->bits);
        bits_skip(&core->bits, 16);
    }

    // Core extensions
    if (core->ext_audio_present) {
        size_t buf_size = (core->bits.total + 31) / 32;
        size_t sync_pos = (core->bits.index + 31) / 32;
        size_t last_pos = core->frame_size / 4;
        if (last_pos > buf_size)
            last_pos = buf_size;

        // Search for extension sync words aligned on 4-byte boundary
        size_t xch_pos = 0, xxch_pos = 0, x96_pos = 0, xbr_pos = 0;

        while (sync_pos < last_pos) {
            size_t hdr_size, dist;

            switch (core->bits.data[sync_pos]) {
            case DCA_32BE(SYNC_WORD_XCH):
                core->bits.index = (sync_pos + 1) * 32;
                hdr_size = bits_get(&core->bits, 10) + 1;
                // XCH comes last after all other extension streams. The
                // distance between XCH sync word and end of the core frame
                // must be equal to XCH frame size. Off by one error is
                // allowed for compatibility with legacy bitstreams.
                dist = core->frame_size - sync_pos * 4;
                if (hdr_size == dist || hdr_size - 1 == dist) {
                    xch_pos = sync_pos + 1;
                    sync_pos = last_pos - 1;
                }
                break;

            case DCA_32BE(SYNC_WORD_XXCH):
                core->bits.index = (sync_pos + 1) * 32;
                hdr_size = bits_get(&core->bits, 6) + 1;
                if (!bits_check_crc(&core->bits, (sync_pos + 1) * 32,
                                    sync_pos * 32 + hdr_size * 8))
                    xxch_pos = sync_pos + 1;
                break;

            case DCA_32BE(SYNC_WORD_X96):
                // X96 comes last after all other extension streams (and can't
                // coexist with XCH apparently). The distance between X96 sync
                // word and end of the core frame must be equal to X96 frame
                // size.
                core->bits.index = (sync_pos + 1) * 32;
                hdr_size = bits_get(&core->bits, 12) + 1;
                dist = core->frame_size - sync_pos * 4;
                if (hdr_size == dist) {
                    x96_pos = sync_pos + 1;
                    sync_pos = last_pos - 1;
                }
                break;

            case DCA_32BE(SYNC_WORD_XBR):
                core->bits.index = (sync_pos + 1) * 32;
                hdr_size = bits_get(&core->bits, 6) + 1;
                if (!bits_check_crc(&core->bits, (sync_pos + 1) * 32,
                                    sync_pos * 32 + hdr_size * 8))
                    xbr_pos = sync_pos + 1;
                break;
            }

            sync_pos++;
        }

        if (xch_pos) {
            //printf("found XCH @ %zu\n", xch_pos);
            core->bits.index = xch_pos * 32;
            if (!parse_xch_frame(core))
                core->xch_present = true;
            else
                revert_to_base_chset(core);
        }

        if (xxch_pos) {
            //printf("found XXCH @ %zu\n", xxch_pos);
            core->bits.index = xxch_pos * 32;
            if (!parse_xxch_frame(core))
                core->xxch_present = true;
            else
                revert_to_base_chset(core);
        }

        if (x96_pos) {
            //printf("found X96 @ %zu\n", x96_pos);
        }

        if (xbr_pos) {
            //printf("found XBR @ %zu\n", xbr_pos);
            core->bits.index = xbr_pos * 32;
            core->xbr_present = !parse_xbr_frame(core);
        }
    }

    return 0;
}

int core_parse(struct core_decoder *core, uint8_t *data, size_t size,
               int flags, struct exss_asset *asset)
{
    core->xch_present = false;
    core->xxch_present = false;
    core->xbr_present = false;

    if (asset) {
        bits_init(&core->bits, data + asset->core_offset, asset->core_size);
        if (bits_get(&core->bits, 32) != SYNC_WORD_CORE_EXSS)
            return -DCADEC_ENOSYNC;
    } else {
        bits_init(&core->bits, data, size);
        bits_skip(&core->bits, 32);
    }

    int ret;
    if ((ret = parse_frame_header(core)) < 0)
        return ret;
    if ((ret = alloc_sample_buffer(core)) < 0)
        return ret;
    if ((ret = parse_frame_data(core, HEADER_CORE, 0)) < 0)
        return ret;
    if ((ret = parse_optional_info(core, flags)) < 0)
        return ret;
    if ((ret = bits_seek(&core->bits, core->frame_size * 8)) < 0)
        return ret;
    return 0;
}

void core_parse_exss(struct core_decoder *core, uint8_t *data, size_t size,
                     int flags, struct exss_asset *asset)
{
    (void)size;
    (void)flags;

    if ((asset->extension_mask & EXSS_XXCH) && !core->xxch_present) {
        bits_init(&core->bits, data + asset->xxch_offset, asset->xxch_size);
        if (bits_get(&core->bits, 32) == SYNC_WORD_XXCH) {
            if (!parse_xxch_frame(core))
                core->xxch_present = true;
            else
                revert_to_base_chset(core);
        }
    }

    if ((asset->extension_mask & EXSS_XBR) && !core->xbr_present) {
        bits_init(&core->bits, data + asset->xbr_offset, asset->xbr_size);
        core->xbr_present =
            bits_get(&core->bits, 32) == SYNC_WORD_XBR && !parse_xbr_frame(core);
    }
}

void core_clear(struct core_decoder *core)
{
    if (core) {
        if (core->subband_buffer) {
            erase_adpcm_history(core);
            for (int n = 0; n < MAX_LFE_HISTORY; n++)
                core->lfe_samples[n] = 0;
        }
        for (int ch = 0; ch < MAX_CHANNELS; ch++)
            interpolator_clear(core->subband_dsp[ch]);
        core->output_history_lfe = 0;
    }
}

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
#include "idct.h"
#include "fixed_math.h"
#include "core_decoder.h"
#include "exss_parser.h"
#include "dmix_tables.h"

#include "core_tables.h"
#include "core_huffman.h"
#include "core_vectors.h"

enum HeaderType {
    HEADER_CORE,
    HEADER_XCH,
    HEADER_XXCH
};

enum AudioMode {
    AMODE_MONO,             // Mode 0: A (mono)
    AMODE_MONO_DUAL,        // Mode 1: A + B (dual mono)
    AMODE_STEREO,           // Mode 2: L + R (stereo)
    AMODE_STEREO_SUMDIFF,   // Mode 3: (L+R) + (L-R) (sum-diff)
    AMODE_STEREO_TOTAL,     // Mode 4: LT + RT (left and right total)
    AMODE_3F,               // Mode 5: C + L + R
    AMODE_2F1R,             // Mode 6: L + R + S
    AMODE_3F1R,             // Mode 7: C + L + R + S
    AMODE_2F2R,             // Mode 8: L + R + SL + SR
    AMODE_3F2R,             // Mode 9: C + L + R + SL + SR

    AMODE_COUNT
};

enum ExtAudioType {
    EXT_AUDIO_XCH   = 0,
    EXT_AUDIO_X96   = 2,
    EXT_AUDIO_XXCH  = 6
};

enum LFEFlag {
    LFE_FLAG_NONE,
    LFE_FLAG_128,
    LFE_FLAG_64,
    LFE_FLAG_INVALID
};

static const int8_t prm_ch_to_spkr_map[AMODE_COUNT][5] = {
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

static const uint8_t audio_mode_ch_mask[AMODE_COUNT] = {
    SPEAKER_LAYOUT_MONO,
    SPEAKER_LAYOUT_STEREO,
    SPEAKER_LAYOUT_STEREO,
    SPEAKER_LAYOUT_STEREO,
    SPEAKER_LAYOUT_STEREO,
    SPEAKER_LAYOUT_3_0,
    SPEAKER_LAYOUT_2_1,
    SPEAKER_LAYOUT_3_1,
    SPEAKER_LAYOUT_2_2,
    SPEAKER_LAYOUT_5POINT0
};

// 5.3.1 - Bit stream header
static int parse_frame_header(struct core_decoder *core)
{
    // Frame type
    bool normal_frame = bits_get1(&core->bits);

    // Deficit sample count
    if (bits_get(&core->bits, 5) != NUM_PCMBLOCK_SAMPLES - 1) {
        core_err("Invalid deficit sample count");
        return normal_frame ? -DCADEC_EBADDATA : -DCADEC_ENOSUP;
    }

    // CRC present flag
    core->crc_present = bits_get1(&core->bits);

    // Number of PCM sample blocks
    core->npcmblocks = bits_get(&core->bits, 7) + 1;
    if (core->npcmblocks & (NUM_SUBBAND_SAMPLES - 1)) {
        core_err("Invalid number of PCM sample blocks (%d)", core->npcmblocks);
        return (core->npcmblocks < 6 || normal_frame) ? -DCADEC_EBADDATA : -DCADEC_ENOSUP;
    }

    // Primary frame byte size
    core->frame_size = bits_get(&core->bits, 14) + 1;
    if (core->frame_size < 96) {
        core_err("Invalid core frame size");
        return -DCADEC_EBADDATA;
    }

    // Audio channel arrangement
    core->audio_mode = bits_get(&core->bits, 6);
    if (core->audio_mode >= AMODE_COUNT) {
        core_err("Unsupported audio channel arrangement (%d)", core->audio_mode);
        return -DCADEC_ENOSUP;
    }

    // Core audio sampling frequency
    core->sample_rate = sample_rates[bits_get(&core->bits, 4)];
    if (!core->sample_rate) {
        core_err("Invalid core audio sampling frequency");
        return -DCADEC_EBADDATA;
    }

    // Transmission bit rate
    core->bit_rate = bit_rates[bits_get(&core->bits, 5)];
    if (core->bit_rate == -1) {
        core_err("Invalid transmission bit rate");
        return -DCADEC_EBADDATA;
    }

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
    core->ext_audio_type = bits_get(&core->bits, 3);

    // Extended coding flag
    core->ext_audio_present = bits_get1(&core->bits);

    // Audio sync word insertion flag
    core->sync_ssf = bits_get1(&core->bits);

    // Low frequency effects flag
    core->lfe_present = bits_get(&core->bits, 2);
    if (core->lfe_present == LFE_FLAG_INVALID) {
        core_err("Invalid low frequency effects flag");
        return -DCADEC_EBADDATA;
    }

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
    if (!core->source_pcm_res) {
        core_err("Invalid source PCM resolution");
        return -DCADEC_EBADDATA;
    }
    core->es_format = !!(pcmr_index & 1);

    // Front sum/difference flag
    core->sumdiff_front = bits_get1(&core->bits);

    // Surround sum/difference flag
    core->sumdiff_surround = bits_get1(&core->bits);

    // Dialog normalization / unspecified
    bits_skip(&core->bits, 4);

    return 0;
}

// 5.3.2 - Primary audio coding header
static int parse_coding_header(struct core_decoder *core, enum HeaderType header, int xch_base)
{
    int ch, n, ret, header_size = 0, header_pos = core->bits.index;

    switch (header) {
    case HEADER_CORE:
        // Number of subframes
        core->nsubframes = bits_get(&core->bits, 4) + 1;

        // Number of primary audio channels
        core->nchannels = bits_get(&core->bits, 3) + 1;
        if (core->nchannels != audio_mode_nch[core->audio_mode]) {
            core_err("Invalid number of primary audio channels (%d) for audio "
                     "channel arrangement (%d)", core->nchannels, core->audio_mode);
            return -DCADEC_EBADDATA;
        }
        assert(core->nchannels <= MAX_CHANNELS - 2);

        core->ch_mask = audio_mode_ch_mask[core->audio_mode];

        // Add LFE channel if present
        if (core->lfe_present)
            core->ch_mask |= SPEAKER_MASK_LFE1;
        break;

    case HEADER_XCH:
        core->nchannels = audio_mode_nch[core->audio_mode] + 1;
        assert(core->nchannels <= MAX_CHANNELS - 1);
        core->ch_mask |= SPEAKER_MASK_Cs;
        break;

    case HEADER_XXCH:
        // Channel set header length
        header_size = bits_get(&core->bits, 7) + 1;

        // Check CRC
        if (core->xxch_crc_present && (ret = bits_check_crc(&core->bits, header_pos, header_pos + header_size * 8)) < 0) {
            core_err("Invalid XXCH channel set header checksum");
            return ret;
        }

        // Number of channels in a channel set
        int nchannels = bits_get(&core->bits, 3) + 1;
        if (nchannels > MAX_CHANNELS_XXCH) {
            core_err_once("Unsupported number of XXCH channels (%d)", nchannels);
            return -DCADEC_ENOSUP;
        }
        core->nchannels = audio_mode_nch[core->audio_mode] + nchannels;
        assert(core->nchannels <= MAX_CHANNELS);

        // Loudspeaker layout mask
        unsigned int mask = bits_get(&core->bits, core->xxch_mask_nbits - SPEAKER_Cs);
        core->xxch_spkr_mask = mask << SPEAKER_Cs;

        if (dca_popcount(core->xxch_spkr_mask) != nchannels) {
            core_err("Invalid XXCH speaker layout mask (%#x)", core->xxch_spkr_mask);
            return -DCADEC_EBADDATA;
        }

        if (core->xxch_core_mask & core->xxch_spkr_mask) {
            core_err("XXCH speaker layout mask (%#x) overlaps with core (%#x)",
                     core->xxch_spkr_mask, core->xxch_core_mask);
            return -DCADEC_EBADDATA;
        }

        // Combine core and XXCH masks together
        core->ch_mask = core->xxch_core_mask | core->xxch_spkr_mask;

        // Downmix coefficients present in stream
        if (bits_get1(&core->bits)) {
            // Downmix already performed by encoder
            core->xxch_dmix_embedded = bits_get1(&core->bits);

            // Downmix scale factor
            unsigned int index = bits_get(&core->bits, 6) * 4 - 44;
            if (index >= dca_countof(dmix_table_inv)) {
                core_err("Invalid XXCH downmix scale index");
                return -DCADEC_EBADDATA;
            }
            core->xxch_dmix_scale_inv = dmix_table_inv[index];

            // Downmix channel mapping mask
            for (ch = 0; ch < nchannels; ch++) {
                mask = bits_get(&core->bits, core->xxch_mask_nbits);
                if ((mask & core->xxch_core_mask) != mask) {
                    core_err("Invalid XXCH downmix channel mapping mask (%#x)", mask);
                    return -DCADEC_EBADDATA;
                }
                core->xxch_dmix_mask[ch] = mask;
            }

            // Downmix coefficients
            int *coeff_ptr = core->xxch_dmix_coeff;
            for (ch = 0; ch < nchannels; ch++) {
                for (n = 0; n < core->xxch_mask_nbits; n++) {
                    if (core->xxch_dmix_mask[ch] & (1U << n)) {
                        int code = bits_get(&core->bits, 7);
                        int sign = (code >> 6) - 1;
                        if (code &= 63) {
                            unsigned int index = code * 4 - 4;
                            if (index >= dca_countof(dmix_table)) {
                                core_err("Invalid XXCH downmix coefficient index");
                                return -DCADEC_EBADDATA;
                            }
                            *coeff_ptr++ = (dmix_table[index] ^ sign) - sign;
                        } else {
                            *coeff_ptr++ = 0;
                        }
                    }
                }
            }
        } else {
            core->xxch_dmix_embedded = false;
        }

        break;
    }

    // Subband activity count
    for (ch = xch_base; ch < core->nchannels; ch++) {
        core->nsubbands[ch] = bits_get(&core->bits, 5) + 2;
        if (core->nsubbands[ch] > MAX_SUBBANDS) {
            core_err("Invalid subband activity count");
            return -DCADEC_EBADDATA;
        }
    }

    // High frequency VQ start subband
    for (ch = xch_base; ch < core->nchannels; ch++)
        core->subband_vq_start[ch] = bits_get(&core->bits, 5) + 1;

    // Joint intensity coding index
    for (ch = xch_base; ch < core->nchannels; ch++) {
        if ((n = bits_get(&core->bits, 3)) && header == HEADER_XXCH)
            n += xch_base - 1;
        if (n > core->nchannels) {
            core_err("Invalid joint intensity coding index");
            return -DCADEC_EBADDATA;
        }
        core->joint_intensity_index[ch] = n;
    }

    // Transient mode code book
    for (ch = xch_base; ch < core->nchannels; ch++)
        core->transition_mode_sel[ch] = bits_get(&core->bits, 2);

    // Scale factor code book
    for (ch = xch_base; ch < core->nchannels; ch++) {
        core->scale_factor_sel[ch] = bits_get(&core->bits, 3);
        if (core->scale_factor_sel[ch] == 7) {
            core_err("Invalid scale factor code book");
            return -DCADEC_EBADDATA;
        }
    }

    // Bit allocation quantizer select
    for (ch = xch_base; ch < core->nchannels; ch++) {
        core->bit_allocation_sel[ch] = bits_get(&core->bits, 3);
        if (core->bit_allocation_sel[ch] == 7) {
            core_err("Invalid bit allocation quantizer select");
            return -DCADEC_EBADDATA;
        }
    }

    // Quantization index codebook select
    for (n = 0; n < NUM_CODE_BOOKS; n++)
        for (ch = xch_base; ch < core->nchannels; ch++)
            core->quant_index_sel[ch][n] = bits_get(&core->bits, quant_index_sel_nbits[n]);

    // Scale factor adjustment index
    for (n = 0; n < NUM_CODE_BOOKS; n++)
        for (ch = xch_base; ch < core->nchannels; ch++)
            if (core->quant_index_sel[ch][n] < quant_index_group_size[n])
                core->scale_factor_adj[ch][n] = scale_factor_adj[bits_get(&core->bits, 2)];

    if (header == HEADER_XXCH) {
        // Reserved
        // Byte align
        // CRC16 of channel set header
        if ((ret = bits_seek(&core->bits, header_pos + header_size * 8)) < 0) {
            core_err("Read past end of XXCH channel set header");
            return ret;
        }
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
    if ((unsigned int)*scale_index >= scale_size) {
        core_err("Invalid scale factor index");
        return -DCADEC_EBADDATA;
    }

    return scale_table[*scale_index];
}

static int parse_joint_scale(struct core_decoder *core, int sel)
{
    int scale_index;

    if (sel < 5)
        scale_index = bits_get_signed_vlc(&core->bits, &scale_factor_huff[sel]);
    else
        scale_index = bits_get(&core->bits, sel + 1);

    // Bias by 64
    scale_index += 64;

    // Look up joint scale factor
    if ((unsigned int)scale_index >= dca_countof(joint_scale_factors)) {
        core_err("Invalid joint scale factor index");
        return -DCADEC_EBADDATA;
    }

    return joint_scale_factors[scale_index];
}

// 5.4.1 - Primary audio coding side information
static int parse_subframe_header(struct core_decoder *core, int sf,
                                 enum HeaderType header, int xch_base)
{
    int ch, band, ret;

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
        // Select codebook
        int sel = core->bit_allocation_sel[ch];
        // Not high frequency VQ subbands
        for (band = 0; band < core->subband_vq_start[ch]; band++) {
            int abits;
            if (sel < 5)
                abits = bits_get_unsigned_vlc(&core->bits, &bit_allocation_huff[sel]) + 1;
            else
                abits = bits_get(&core->bits, sel - 1);
            if (abits >= 27) {
                core_err("Invalid bit allocation index");
                return -DCADEC_EBADDATA;
            }
            core->bit_allocation[ch][band] = abits;
        }
    }

    // Transition mode
    for (ch = xch_base; ch < core->nchannels; ch++) {
        // Clear transition mode for all subbands
        memset(core->transition_mode[sf][ch], 0, sizeof(core->transition_mode[0][0]));

        // Transient possible only if more than one subsubframe
        if (core->nsubsubframes[sf] > 1) {
            // Select codebook
            int sel = core->transition_mode_sel[ch];
            // Not high frequency VQ subbands
            for (band = 0; band < core->subband_vq_start[ch]; band++) {
                // Present only if bits allocated
                if (core->bit_allocation[ch][band]) {
                    int trans_ssf = bits_get_unsigned_vlc(&core->bits, &transition_mode_huff[sel]);
                    if (trans_ssf >= 4) {
                        core_err("Invalid transition mode index");
                        return -DCADEC_EBADDATA;
                    }
                    core->transition_mode[sf][ch][band] = trans_ssf;
                }
            }
        }
    }

    // Scale factors
    for (ch = xch_base; ch < core->nchannels; ch++) {
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
            } else {
                core->scale_factors[ch][band][0] = 0;
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
            if (core->joint_scale_sel[ch] == 7) {
                core_err("Invalid joint scale factor code book");
                return -DCADEC_EBADDATA;
            }
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
                if ((ret = parse_joint_scale(core, sel)) < 0)
                    return ret;
                core->joint_scale_factors[ch][band] = ret;
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

static int parse_block_codes(struct core_decoder *core, int *audio, int abits)
{
    // Extract the block code indices from the bit stream
    int code1 = bits_get(&core->bits, block_code_nbits[abits]);
    int code2 = bits_get(&core->bits, block_code_nbits[abits]);
    int levels = quant_levels[abits];
    int offset = (levels - 1) / 2;
    int n;

    // Look up samples from the block code book
    for (n = 0; n < NUM_SUBBAND_SAMPLES / 2; n++) {
        audio[n] = (code1 % levels) - offset;
        code1 /= levels;
    }
    for (; n < NUM_SUBBAND_SAMPLES; n++) {
        audio[n] = (code2 % levels) - offset;
        code2 /= levels;
    }

    if (code1 || code2) {
        core_err("Failed to decode block codes");
        return -DCADEC_EBADDATA;
    }

    return 0;
}

static int parse_huffman_codes(struct core_decoder *core, int *audio, int abits, int sel)
{
    int ret;

    // Extract Huffman codes from the bit stream
    if ((ret = bits_get_signed_vlc_array(&core->bits, audio, NUM_SUBBAND_SAMPLES,
                                         &quant_index_group_huff[abits - 1][sel])) < 0) {
        core_err("Failed to decode huffman codes");
        return ret;
    }

    return 1;
}

static inline int extract_audio(struct core_decoder *core, int *audio, int abits, int ch)
{
    assert(abits >= 0 && abits < 27);

    if (abits == 0) {
        // No bits allocated
        memset(audio, 0, NUM_SUBBAND_SAMPLES * sizeof(*audio));
        return 0;
    }

    if (abits <= NUM_CODE_BOOKS) {
        int sel = core->quant_index_sel[ch][abits - 1];
        if (sel < quant_index_group_size[abits - 1]) {
            // Huffman codes
            return parse_huffman_codes(core, audio, abits, sel);
        }
        if (abits <= 7) {
            // Block codes
            return parse_block_codes(core, audio, abits);
        }
    }

    // No further encoding
    bits_get_signed_array(&core->bits, audio, NUM_SUBBAND_SAMPLES, abits - 3);
    return 0;
}

static inline void dequantize(int *output, const int *input, int step_size,
                              int scale, bool residual)
{
    // Account for quantizer step size
    int64_t step_scale = (int64_t)step_size * scale;
    int shift = 0;

    // Limit scale factor resolution to 22 bits
    if (step_scale > (1 << 23)) {
        shift = 32 - dca_clz(step_scale >> 23);
        step_scale >>= shift;
    }

    // Scale the samples
    if (residual) {
        for (int n = 0; n < NUM_SUBBAND_SAMPLES; n++)
            output[n] += clip23(norm__(input[n] * step_scale, 22 - shift));
    } else {
        for (int n = 0; n < NUM_SUBBAND_SAMPLES; n++)
            output[n]  = clip23(norm__(input[n] * step_scale, 22 - shift));
    }
}

// 5.5 - Primary audio data arrays
static int parse_subframe_audio(struct core_decoder *core, int sf, enum HeaderType header,
                                int xch_base, int *sub_pos, int *lfe_pos)
{
    int ssf, ch, band, ofs;

    // Number of subband samples in this subframe
    int nsamples = core->nsubsubframes[sf] * NUM_SUBBAND_SAMPLES;
    if (*sub_pos + nsamples > core->npcmblocks) {
        core_err("Subband sample buffer overflow");
        return -DCADEC_EBADDATA;
    }

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
        if (scale_index >= dca_countof(scale_factors_7bit)) {
            core_err("Invalid LFE scale factor index");
            return -DCADEC_EBADDATA;
        }

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
    for (ssf = 0, ofs = *sub_pos; ssf < core->nsubsubframes[sf]; ssf++) {
        for (ch = xch_base; ch < core->nchannels; ch++) {
            // Not high frequency VQ subbands
            for (band = 0; band < core->subband_vq_start[ch]; band++) {
                int abits = core->bit_allocation[ch][band];
                int audio[NUM_SUBBAND_SAMPLES];
                int ret, step_size, trans_ssf, scale;

                // Extract bits from the bit stream
                if ((ret = extract_audio(core, audio, abits, ch)) < 0)
                    return ret;

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
                if (ret > 0) {
                    int64_t adj = core->scale_factor_adj[ch][abits - 1];
                    scale = clip23((adj * scale) >> 22);
                }

                dequantize(core->subband_samples[ch][band] + ofs,
                           audio, step_size, scale, false);
            }
        }

        // DSYNC
        if ((ssf == core->nsubsubframes[sf] - 1 || core->sync_ssf)
            && bits_get(&core->bits, 16) != 0xffff) {
            core_err("DSYNC check failed");
            return -DCADEC_EBADDATA;
        }

        ofs += NUM_SUBBAND_SAMPLES;
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
    for (int ch = 0; ch < MAX_CHANNELS; ch++)
        for (int band = 0; band < MAX_SUBBANDS; band++)
            memset(core->subband_samples[ch][band] - NUM_ADPCM_COEFFS, 0, NUM_ADPCM_COEFFS * sizeof(int));
}

static int alloc_sample_buffer(struct core_decoder *core)
{
    int nchsamples = NUM_ADPCM_COEFFS + core->npcmblocks;
    int nframesamples = nchsamples * MAX_CHANNELS * MAX_SUBBANDS;
    int nlfesamples = MAX_LFE_HISTORY + core->npcmblocks / 2;

    // Reallocate subband sample buffer
    int ret;
    if ((ret = ta_zalloc_fast(core, &core->subband_buffer, nframesamples + nlfesamples, sizeof(int))) < 0)
        return -DCADEC_ENOMEM;
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

static int parse_frame_data(struct core_decoder *core, enum HeaderType header, int xch_base)
{
    int ret;
    if ((ret = parse_coding_header(core, header, xch_base)) < 0)
        return ret;

    int sub_pos = 0;
    int lfe_pos = MAX_LFE_HISTORY;
    for (int sf = 0; sf < core->nsubframes; sf++) {
        if ((ret = parse_subframe_header(core, sf, header, xch_base)) < 0)
            return ret;
        if ((ret = parse_subframe_audio(core, sf, header, xch_base, &sub_pos, &lfe_pos)) < 0)
            return ret;
    }

    for (int ch = xch_base; ch < core->nchannels; ch++) {
        // Number of active subbands for this channel
        int nsubbands = core->nsubbands[ch];
        if (core->joint_intensity_index[ch])
            nsubbands = DCA_MAX(nsubbands, core->nsubbands[core->joint_intensity_index[ch] - 1]);

        // Update history for ADPCM
        for (int band = 0; band < nsubbands; band++) {
            int *samples = core->subband_samples[ch][band] - NUM_ADPCM_COEFFS;
            memcpy(samples, samples + core->npcmblocks, NUM_ADPCM_COEFFS * sizeof(int));
        }

        // Clear inactive subbands
        for (int band = nsubbands; band < MAX_SUBBANDS; band++) {
            int *samples = core->subband_samples[ch][band] - NUM_ADPCM_COEFFS;
            memset(samples, 0, (NUM_ADPCM_COEFFS + core->npcmblocks) * sizeof(int));
        }
    }

    return 0;
}

static int map_prm_ch_to_spkr(struct core_decoder *core, int ch)
{
    // Try to map this channel to core first
    int pos = audio_mode_nch[core->audio_mode];
    if (ch < pos) {
        int spkr = prm_ch_to_spkr_map[core->audio_mode][ch];
        if (core->ext_audio_mask & (CSS_XXCH | EXSS_XXCH)) {
            if (core->xxch_core_mask & (1U << spkr))
                return spkr;
            if (spkr == SPEAKER_Ls && (core->xxch_core_mask & SPEAKER_MASK_Lss))
                return SPEAKER_Lss;
            if (spkr == SPEAKER_Rs && (core->xxch_core_mask & SPEAKER_MASK_Rss))
                return SPEAKER_Rss;
            return -1;
        }
        return spkr;
    }

    // Then XCH
    if ((core->ext_audio_mask & CSS_XCH) && ch == pos)
        return SPEAKER_Cs;

    // Then XXCH
    if (core->ext_audio_mask & (CSS_XXCH | EXSS_XXCH))
        for (int spkr = SPEAKER_Cs; spkr < core->xxch_mask_nbits; spkr++)
            if (core->xxch_spkr_mask & (1U << spkr))
                if (pos++ == ch)
                    return spkr;

    // No mapping
    return -1;
}

int core_filter(struct core_decoder *core, int flags)
{
    int x96_nchannels = 0;

    // Externally set CORE_SYNTH_X96 flags implies that X96 synthesis should be
    // enabled, yet actual X96 subband data should be discarded. This is a special
    // case for lossless residual decoder that apparently ignores X96 data.
    if (!(flags & DCADEC_FLAG_CORE_SYNTH_X96) && (core->ext_audio_mask & (CSS_X96 | EXSS_X96))) {
        x96_nchannels = core->x96_nchannels;
        flags |= DCADEC_FLAG_CORE_SYNTH_X96;
    }

    // X96 synthesis enabled flag
    bool synth_x96 = !!(flags & DCADEC_FLAG_CORE_SYNTH_X96);

    // Output sample rate
    core->output_rate = core->sample_rate << synth_x96;

    // Number of PCM samples in this frame
    core->npcmsamples = (core->npcmblocks * NUM_PCMBLOCK_SAMPLES) << synth_x96;

    // Reallocate PCM output buffer
    if (ta_zalloc_fast(core, &core->output_buffer, core->npcmsamples * dca_popcount(core->ch_mask), sizeof(int)) < 0)
        return -DCADEC_ENOMEM;

    int *ptr = core->output_buffer;
    for (int spkr = 0; spkr < SPEAKER_COUNT; spkr++) {
        if (core->ch_mask & (1U << spkr)) {
            core->output_samples[spkr] = ptr;
            ptr += core->npcmsamples;
        } else {
            core->output_samples[spkr] = NULL;
        }
    }

    // Handle change of certain filtering parameters
    int diff = core->filter_flags ^ flags;

    if (diff & (DCADEC_FLAG_CORE_BIT_EXACT | DCADEC_FLAG_CORE_SYNTH_X96)) {
        for (int ch = 0; ch < MAX_CHANNELS; ch++) {
            ta_free(core->subband_dsp[ch]);
            core->subband_dsp[ch] = NULL;
        }
    }

    if (diff & (DCADEC_FLAG_CORE_BIT_EXACT | DCADEC_FLAG_CORE_LFE_IIR))
        memset(core->lfe_samples, 0, MAX_LFE_HISTORY * sizeof(int));

    if (diff & DCADEC_FLAG_CORE_SYNTH_X96)
        core->output_history_lfe = 0;

    core->filter_flags = flags;

    if (!core->subband_dsp_idct[synth_x96] && !(core->subband_dsp_idct[synth_x96] = idct_init(core, 5 + synth_x96, 0.25)))
        return -DCADEC_ENOMEM;

    // Filter primary channels
    for (int ch = 0; ch < core->nchannels; ch++) {
        // Allocate subband DSP
        if (!core->subband_dsp[ch] && !(core->subband_dsp[ch] = interpolator_create(core->subband_dsp_idct[synth_x96], flags)))
            return -DCADEC_ENOMEM;

        // Map this primary channel to speaker
        int spkr = map_prm_ch_to_spkr(core, ch);
        if (spkr < 0)
            return -DCADEC_EINVAL;

        // Get the pointer to high frequency subbands for this channel, if present
        int **subband_samples_hi;
        if (ch < x96_nchannels)
            subband_samples_hi = core->x96_subband_samples[ch];
        else
            subband_samples_hi = NULL;

        // Filter bank reconstruction
        core->subband_dsp[ch]->interpolate(core->subband_dsp[ch],
                                           core->output_samples[spkr],
                                           core->subband_samples[ch],
                                           subband_samples_hi,
                                           core->npcmblocks,
                                           core->filter_perfect);
    }

    // Filter LFE channel
    if (core->lfe_present) {
        bool dec_select = (core->lfe_present == LFE_FLAG_128);
        interpolate_lfe_cb interpolate;

        // Select LFE DSP
        if (flags & DCADEC_FLAG_CORE_BIT_EXACT) {
            if (dec_select) {
                core_err("Fixed point mode doesn't support LFF=1");
                return -DCADEC_EINVAL;
            }
            interpolate = interpolate_lfe_fixed_fir;
        } else if (flags & DCADEC_FLAG_CORE_LFE_IIR) {
            interpolate = interpolate_lfe_float_iir;
        } else if (dec_select) {
            interpolate = interpolate_lfe_float_fir_2x;
        } else {
            interpolate = interpolate_lfe_float_fir;
        }

        // Offset output buffer for X96
        int *samples = core->output_samples[SPEAKER_LFE1];
        if (synth_x96)
            samples += core->npcmsamples / 2;

        // Interpolation of LFE channel
        interpolate(samples, core->lfe_samples, core->npcmblocks, dec_select);

        if (synth_x96) {
            // Filter 96 kHz oversampled LFE PCM to attenuate high frequency
            // (47.6 - 48.0 kHz) components of interpolation image
            int history = core->output_history_lfe;
            int *samples2 = core->output_samples[SPEAKER_LFE1];
            int nsamples = core->npcmsamples / 2;
            for (int n = 0; n < nsamples; n++) {
                int64_t res1 = INT64_C(2097471) * samples[n] + INT64_C(6291137) * history;
                int64_t res2 = INT64_C(6291137) * samples[n] + INT64_C(2097471) * history;
                history = samples[n];
                samples2[2 * n    ] = clip23(norm23(res1));
                samples2[2 * n + 1] = clip23(norm23(res2));
            }

            // Update LFE PCM history
            core->output_history_lfe = history;
        }
    }

    if (!(flags & DCADEC_FLAG_KEEP_DMIX_MASK)) {
        int nsamples = core->npcmsamples;

        // Undo embedded XCH downmix
        if (core->es_format && (core->ext_audio_mask & CSS_XCH) && core->audio_mode >= AMODE_2F2R) {
            int *samples_ls = core->output_samples[SPEAKER_Ls];
            int *samples_rs = core->output_samples[SPEAKER_Rs];
            int *samples_cs = core->output_samples[SPEAKER_Cs];
            for (int n = 0; n < nsamples; n++) {
                int cs = mul23(samples_cs[n], 5931520);
                samples_ls[n] = clip23(samples_ls[n] - cs);
                samples_rs[n] = clip23(samples_rs[n] - cs);
            }
        }

        // Undo embedded XXCH downmix
        if ((core->ext_audio_mask & (CSS_XXCH | EXSS_XXCH)) && core->xxch_dmix_embedded) {
            int xch_base = audio_mode_nch[core->audio_mode];
            assert(core->nchannels - xch_base <= MAX_CHANNELS_XXCH);

            // Undo embedded core downmix pre-scaling
            int scale_inv = core->xxch_dmix_scale_inv;
            if (scale_inv != (1 << 16)) {
                for (int spkr = 0; spkr < core->xxch_mask_nbits; spkr++) {
                    if (core->xxch_core_mask & (1U << spkr)) {
                        int *samples = core->output_samples[spkr];
                        for (int n = 0; n < nsamples; n++)
                            samples[n] = mul16(samples[n], scale_inv);
                    }
                }
            }

            // Undo downmix
            int *coeff_ptr = core->xxch_dmix_coeff;
            for (int ch = xch_base; ch < core->nchannels; ch++) {
                int spkr1 = map_prm_ch_to_spkr(core, ch);
                if (spkr1 < 0)
                    return -DCADEC_EINVAL;
                for (int spkr2 = 0; spkr2 < core->xxch_mask_nbits; spkr2++) {
                    if (core->xxch_dmix_mask[ch - xch_base] & (1U << spkr2)) {
                        int coeff = mul16(*coeff_ptr++, scale_inv);
                        if (coeff) {
                            int *src = core->output_samples[spkr1];
                            int *dst = core->output_samples[spkr2];
                            for (int n = 0; n < nsamples; n++)
                                dst[n] -= mul15(src[n], coeff);
                        }
                    }
                }
            }

            // Clip core channels
            for (int spkr = 0; spkr < core->xxch_mask_nbits; spkr++) {
                if (core->xxch_core_mask & (1U << spkr)) {
                    int *samples = core->output_samples[spkr];
                    for (int n = 0; n < nsamples; n++)
                        samples[n] = clip23(samples[n]);
                }
            }
        }
    }

    if (!(core->ext_audio_mask & (CSS_XXCH | CSS_XCH | EXSS_XXCH))) {
        int nsamples = core->npcmsamples;

        // Front sum/difference decoding
        if ((core->sumdiff_front && core->audio_mode > AMODE_MONO)
            || core->audio_mode == AMODE_STEREO_SUMDIFF) {
            int *samples_l = core->output_samples[SPEAKER_L];
            int *samples_r = core->output_samples[SPEAKER_R];
            for (int n = 0; n < nsamples; n++) {
                int res1 = samples_l[n] + samples_r[n];
                int res2 = samples_l[n] - samples_r[n];
                samples_l[n] = clip23(res1);
                samples_r[n] = clip23(res2);
            }
        }

        // Surround sum/difference decoding
        if (core->sumdiff_surround && core->audio_mode >= AMODE_2F2R) {
            int *samples_ls = core->output_samples[SPEAKER_Ls];
            int *samples_rs = core->output_samples[SPEAKER_Rs];
            for (int n = 0; n < nsamples; n++) {
                int res1 = samples_ls[n] + samples_rs[n];
                int res2 = samples_ls[n] - samples_rs[n];
                samples_ls[n] = clip23(res1);
                samples_rs[n] = clip23(res2);
            }
        }
    }

    return 0;
}

static int parse_xch_frame(struct core_decoder *core)
{
    if (core->ch_mask & SPEAKER_MASK_Cs) {
        core_err("XCH with Cs speaker already present");
        return -DCADEC_EBADDATA;
    }

    int ret;
    if ((ret = parse_frame_data(core, HEADER_XCH, core->nchannels)) < 0)
        return ret;

    // Seek to the end of core frame, don't trust XCH frame size
    return bits_seek(&core->bits, core->frame_size * 8);
}

static int parse_xxch_frame(struct core_decoder *core)
{
    int header_pos = core->bits.index;

    // XXCH sync word
    if (bits_get(&core->bits, 32) != SYNC_WORD_XXCH) {
        core_err("Invalid XXCH sync word");
        return -DCADEC_ENOSYNC;
    }

    // XXCH frame header length
    int header_size = bits_get(&core->bits, 6) + 1;

    // Check XXCH frame header CRC
    int ret;
    if ((ret = bits_check_crc(&core->bits, header_pos + 32, header_pos + header_size * 8)) < 0) {
        core_err("Invalid XXCH frame header checksum");
        return ret;
    }

    // CRC presence flag for channel set header
    core->xxch_crc_present = bits_get1(&core->bits);

    // Number of bits for loudspeaker mask
    core->xxch_mask_nbits = bits_get(&core->bits, 5) + 1;
    if (core->xxch_mask_nbits <= SPEAKER_Cs) {
        core_err("Invalid number of bits for XXCH speaker mask (%d)", core->xxch_mask_nbits);
        return -DCADEC_EBADDATA;
    }

    // Number of channel sets
    int xxch_nchsets = bits_get(&core->bits, 2) + 1;
    if (xxch_nchsets > 1) {
        core_err_once("Unsupported number of XXCH channel sets (%d)", xxch_nchsets);
        return -DCADEC_ENOSUP;
    }

    // Channel set 0 data byte size
    int xxch_frame_size = bits_get(&core->bits, 14) + 1;

    // Core loudspeaker activity mask
    core->xxch_core_mask = bits_get(&core->bits, core->xxch_mask_nbits);

    // Validate the core mask
    int mask = core->ch_mask;

    if ((mask & SPEAKER_MASK_Ls) && (core->xxch_core_mask & SPEAKER_MASK_Lss))
        mask = (mask & ~SPEAKER_MASK_Ls) | SPEAKER_MASK_Lss;

    if ((mask & SPEAKER_MASK_Rs) && (core->xxch_core_mask & SPEAKER_MASK_Rss))
        mask = (mask & ~SPEAKER_MASK_Rs) | SPEAKER_MASK_Rss;

    if (mask != core->xxch_core_mask) {
        core_err("XXCH core speaker activity mask (%#x) disagrees "
                 "with core (%#x)", core->xxch_core_mask, mask);
        return -DCADEC_EBADDATA;
    }

    // Reserved
    // Byte align
    // CRC16 of XXCH frame header
    if ((ret = bits_seek(&core->bits, header_pos + header_size * 8)) < 0) {
        core_err("Read past end of XXCH frame header");
        return ret;
    }

    // Parse XXCH channel set 0
    if ((ret = parse_frame_data(core, HEADER_XXCH, core->nchannels)) < 0)
        return ret;

    if ((ret = bits_seek(&core->bits, header_pos + header_size * 8 + xxch_frame_size * 8)) < 0)
        core_err("Read past end of XXCH channel set");
    return ret;
}

static int parse_xbr_subframe(struct core_decoder *core, int xbr_base_ch, int xbr_nchannels,
                              int *xbr_nsubbands, bool xbr_transition_mode, int sf, int *sub_pos)
{
    int     xbr_nabits[MAX_CHANNELS];
    int     xbr_bit_allocation[MAX_CHANNELS][MAX_SUBBANDS];
    int     xbr_scale_nbits[MAX_CHANNELS];
    int     xbr_scale_factors[MAX_CHANNELS][MAX_SUBBANDS][2];
    int     ssf, ch, band, ofs;

    // Number of subband samples in this subframe
    int nsamples = core->nsubsubframes[sf] * NUM_SUBBAND_SAMPLES;
    if (*sub_pos + nsamples > core->npcmblocks) {
        core_err("Subband sample buffer overflow");
        return -DCADEC_EBADDATA;
    }

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
        if (!xbr_scale_nbits[ch]) {
            core_err("Invalid number of bits for XBR scale factor index");
            return -DCADEC_EBADDATA;
        }
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
                if (scale_index >= scale_size) {
                    core_err("Invalid XBR scale factor index");
                    return -DCADEC_EBADDATA;
                }
                xbr_scale_factors[ch][band][0] = scale_table[scale_index];
                if (xbr_transition_mode && core->transition_mode[sf][ch][band]) {
                    scale_index = bits_get(&core->bits, xbr_scale_nbits[ch]);
                    if (scale_index >= scale_size) {
                        core_err("Invalid XBR scale factor index");
                        return -DCADEC_EBADDATA;
                    }
                    xbr_scale_factors[ch][band][1] = scale_table[scale_index];
                }
            }
        }
    }

    // Audio data
    for (ssf = 0, ofs = *sub_pos; ssf < core->nsubsubframes[sf]; ssf++) {
        for (ch = xbr_base_ch; ch < xbr_nchannels; ch++) {
            for (band = 0; band < xbr_nsubbands[ch]; band++) {
                int abits = xbr_bit_allocation[ch][band];
                int audio[NUM_SUBBAND_SAMPLES];
                int ret, step_size, trans_ssf, scale;

                // Extract bits from the bit stream
                if (abits > 7) {
                    // No further encoding
                    bits_get_signed_array(&core->bits, audio, NUM_SUBBAND_SAMPLES, abits - 3);
                } else if (abits > 0) {
                    // Block codes
                    if ((ret = parse_block_codes(core, audio, abits)) < 0)
                        return ret;
                } else {
                    // No bits allocated
                    continue;
                }

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

                dequantize(core->subband_samples[ch][band] + ofs,
                           audio, step_size, scale, true);
            }
        }

        // DSYNC
        if ((ssf == core->nsubsubframes[sf] - 1 || core->sync_ssf)
            && bits_get(&core->bits, 16) != 0xffff) {
            core_err("XBR-DSYNC check failed");
            return -DCADEC_EBADDATA;
        }

        ofs += NUM_SUBBAND_SAMPLES;
    }

    // Advance subband sample pointer for the next subframe
    *sub_pos += nsamples;
    return 0;
}

static int parse_xbr_frame(struct core_decoder *core)
{
    int     xbr_frame_size[MAX_EXSS_CHSETS];
    int     xbr_nchannels[MAX_EXSS_CHSETS];
    int     xbr_nsubbands[MAX_EXSS_CHSETS * MAX_CHANNELS_CHSET];

    int header_pos = core->bits.index;

    // XBR sync word
    if (bits_get(&core->bits, 32) != SYNC_WORD_XBR) {
        core_err("Invalid XBR sync word");
        return -DCADEC_ENOSYNC;
    }

    // XBR frame header length
    int header_size = bits_get(&core->bits, 6) + 1;

    // Check XBR frame header CRC
    int ret;
    if ((ret = bits_check_crc(&core->bits, header_pos + 32, header_pos + header_size * 8)) < 0) {
        core_err("Invalid XBR frame header checksum");
        return ret;
    }

    // Number of channel sets
    int xbr_nchsets = bits_get(&core->bits, 2) + 1;

    // Channel set data byte size
    for (int i = 0; i < xbr_nchsets; i++)
        xbr_frame_size[i] = bits_get(&core->bits, 14) + 1;

    // Transition mode flag
    bool xbr_transition_mode = bits_get1(&core->bits);

    // Channel set headers
    for (int i = 0, ch2 = 0; i < xbr_nchsets; i++) {
        xbr_nchannels[i] = bits_get(&core->bits, 3) + 1;
        int xbr_band_nbits = bits_get(&core->bits, 2) + 5;
        for (int ch1 = 0; ch1 < xbr_nchannels[i]; ch1++, ch2++) {
            xbr_nsubbands[ch2] = bits_get(&core->bits, xbr_band_nbits) + 1;
            if (xbr_nsubbands[ch2] > MAX_SUBBANDS) {
                core_err("Invalid number of active XBR subbands (%d)", xbr_nsubbands[ch2]);
                return -DCADEC_EBADDATA;
            }
        }
    }

    // Reserved
    // Byte align
    // CRC16 of XBR frame header
    if ((ret = bits_seek(&core->bits, header_pos + header_size * 8)) < 0) {
        core_err("Read past end of XBR frame header");
        return ret;
    }

    // Channel set data
    int xbr_base_ch = 0;
    for (int i = 0; i < xbr_nchsets; i++) {
        header_pos = core->bits.index;

        if (xbr_base_ch + xbr_nchannels[i] <= core->nchannels) {
            int sub_pos = 0;
            for (int sf = 0; sf < core->nsubframes; sf++) {
                if ((ret = parse_xbr_subframe(core, xbr_base_ch,
                                              xbr_base_ch + xbr_nchannels[i],
                                              xbr_nsubbands, xbr_transition_mode,
                                              sf, &sub_pos)) < 0)
                    return ret;
            }
        }

        xbr_base_ch += xbr_nchannels[i];

        if ((ret = bits_seek(&core->bits, header_pos + xbr_frame_size[i] * 8)) < 0) {
            core_err("Read past end of XBR channel set");
            return ret;
        }
    }

    return 0;
}

// Modified ISO/IEC 9899 linear congruential generator
// Returns pseudorandom integer in range [-2^30, 2^30 - 1]
static int rand_x96(struct core_decoder *core)
{
    core->x96_rand = 1103515245U * core->x96_rand + 12345U;
    return (core->x96_rand & 0x7fffffff) - 0x40000000;
}

static int parse_x96_subframe_audio(struct core_decoder *core, int sf, int xch_base, int *sub_pos)
{
    int ssf, ch, band, ofs;

    // Number of subband samples in this subframe
    int nsamples = core->nsubsubframes[sf] * NUM_SUBBAND_SAMPLES;
    if (*sub_pos + nsamples > core->npcmblocks) {
        core_err("Subband sample buffer overflow");
        return -DCADEC_EBADDATA;
    }

    // VQ encoded or unallocated subbands
    for (ch = xch_base; ch < core->x96_nchannels; ch++) {
        for (band = core->x96_subband_start; band < core->nsubbands[ch]; band++) {
            // Get the sample pointer
            int *samples = core->x96_subband_samples[ch][band] + *sub_pos;

            // Get the scale factor
            int scale = core->x96_scale_factors[ch][band];

            int abits = core->bit_allocation[ch][band];
            if (abits == 0) {   // No bits allocated for subband
                if (scale <= 1) {
                    memset(samples, 0, nsamples * sizeof(int));
                } else {
                    // Generate scaled random samples as required by specification
                    for (int n = 0; n < nsamples; n++)
                        samples[n] = mul31(rand_x96(core), scale);
                }
            } else if (abits == 1) {    // VQ encoded subband
                for (ssf = 0; ssf < (core->nsubsubframes[sf] + 1) / 2; ssf++) {
                    // Extract the VQ address from the bit stream
                    int vq_index = bits_get(&core->bits, 10);

                    // Look up the VQ code book for up to 16 subband samples
                    const int8_t *vq_samples = high_freq_samples[vq_index];

                    // Number of VQ samples to look up
                    int vq_nsamples = DCA_MIN(nsamples - ssf * 16, 16);

                    // Scale and take the samples
                    for (int n = 0; n < vq_nsamples; n++)
                        *samples++ = clip23(mul4(scale, vq_samples[n]));
                }
            }
        }
    }

    // Audio data
    for (ssf = 0, ofs = *sub_pos; ssf < core->nsubsubframes[sf]; ssf++) {
        for (ch = xch_base; ch < core->x96_nchannels; ch++) {
            for (band = core->x96_subband_start; band < core->nsubbands[ch]; band++) {
                int abits = core->bit_allocation[ch][band] - 1;
                int audio[NUM_SUBBAND_SAMPLES];
                int ret, step_size, scale;

                // Not VQ encoded or unallocated subbands
                if (abits < 1)
                    continue;

                // Extract bits from the bit stream
                if ((ret = extract_audio(core, audio, abits, ch)) < 0)
                    return ret;

                // Select quantization step size table
                // Look up quantization step size
                if (core->bit_rate == -2)
                    step_size = step_size_lossless[abits];
                else
                    step_size = step_size_lossy[abits];

                // Determine proper scale factor
                scale = core->x96_scale_factors[ch][band];

                dequantize(core->x96_subband_samples[ch][band] + ofs,
                           audio, step_size, scale, false);
            }
        }

        // DSYNC
        if ((ssf == core->nsubsubframes[sf] - 1 || core->sync_ssf)
            && bits_get(&core->bits, 16) != 0xffff) {
            core_err("X96-DSYNC check failed");
            return -DCADEC_EBADDATA;
        }

        ofs += NUM_SUBBAND_SAMPLES;
    }

    // Inverse ADPCM
    for (ch = xch_base; ch < core->x96_nchannels; ch++) {
        for (band = core->x96_subband_start; band < core->nsubbands[ch]; band++) {
            // Only if prediction mode is on
            if (core->prediction_mode[ch][band]) {
                int *samples = core->x96_subband_samples[ch][band] + *sub_pos;

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
    for (ch = xch_base; ch < core->x96_nchannels; ch++) {
        // Only if joint subband coding is enabled
        if (core->joint_intensity_index[ch]) {
            // Get source channel
            int src_ch = core->joint_intensity_index[ch] - 1;
            for (band = core->nsubbands[ch]; band < core->nsubbands[src_ch]; band++) {
                int *src = core->x96_subband_samples[src_ch][band] + *sub_pos;
                int *dst = core->x96_subband_samples[    ch][band] + *sub_pos;
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

static void erase_x96_adpcm_history(struct core_decoder *core)
{
    // Erase ADPCM history from previous frame if
    // predictor history switch was disabled
    for (int ch = 0; ch < MAX_CHANNELS; ch++)
        for (int band = 0; band < MAX_SUBBANDS_X96; band++)
            memset(core->x96_subband_samples[ch][band] - NUM_ADPCM_COEFFS, 0, NUM_ADPCM_COEFFS * sizeof(int));
}

static int alloc_x96_sample_buffer(struct core_decoder *core)
{
    int nchsamples = NUM_ADPCM_COEFFS + core->npcmblocks;
    int nframesamples = nchsamples * MAX_CHANNELS * MAX_SUBBANDS_X96;

    // Reallocate subband sample buffer
    int ret;
    if ((ret = ta_zalloc_fast(core, &core->x96_subband_buffer, nframesamples, sizeof(int))) < 0)
        return -DCADEC_ENOMEM;
    if (ret > 0) {
        for (int ch = 0; ch < MAX_CHANNELS; ch++)
            for (int band = 0; band < MAX_SUBBANDS_X96; band++)
                core->x96_subband_samples[ch][band] = core->x96_subband_buffer +
                    (ch * MAX_SUBBANDS_X96 + band) * nchsamples + NUM_ADPCM_COEFFS;
    }

    if (!core->predictor_history)
        erase_x96_adpcm_history(core);

    return 0;
}

static int parse_x96_subframe_header(struct core_decoder *core, int xch_base)
{
    int ch, band, ret;

    // Prediction mode
    for (ch = xch_base; ch < core->x96_nchannels; ch++)
        for (band = core->x96_subband_start; band < core->nsubbands[ch]; band++)
            core->prediction_mode[ch][band] = bits_get1(&core->bits);

    // Prediction coefficients VQ address
    for (ch = xch_base; ch < core->x96_nchannels; ch++)
        for (band = core->x96_subband_start; band < core->nsubbands[ch]; band++)
            if (core->prediction_mode[ch][band])
                core->prediction_vq_index[ch][band] = bits_get(&core->bits, 12);

    // Bit allocation index
    for (ch = xch_base; ch < core->x96_nchannels; ch++) {
        // Select codebook
        int sel = core->bit_allocation_sel[ch];

        const struct huffman *huff;
        unsigned int abits_max;

        // Reuse quantization index code books for bit allocation index
        if (core->x96_high_res) {
            huff = &quant_index_huff_7[sel];
            abits_max = 15;
        } else {
            huff = &quant_index_huff_5[sel];
            abits_max = 7;
        }

        // Clear accumulation
        int abits = 0;

        for (band = core->x96_subband_start; band < core->nsubbands[ch]; band++) {
            if (sel < 7)
                // If Huffman code was used, the difference of abits was encoded
                abits += bits_get_signed_vlc(&core->bits, huff);
            else
                abits = bits_get(&core->bits, 3 + core->x96_high_res);

            if ((unsigned int)abits > abits_max) {
                core_err("Invalid X96 bit allocation index");
                return -DCADEC_EBADDATA;
            }

            core->bit_allocation[ch][band] = abits;
        }
    }

    // Scale factors
    for (ch = xch_base; ch < core->x96_nchannels; ch++) {
        // Select codebook
        int sel = core->scale_factor_sel[ch];

        // Clear accumulation
        int scale_index = 0;

        // Extract scales for subbands
        // Transmitted even for unallocated subbands
        for (band = core->x96_subband_start; band < core->nsubbands[ch]; band++) {
            if ((ret = parse_scale(core, &scale_index, sel)) < 0)
                return ret;
            core->x96_scale_factors[ch][band] = ret;
        }
    }

    // Joint subband codebook select
    for (ch = xch_base; ch < core->x96_nchannels; ch++) {
        // Only if joint subband coding is enabled
        if (core->joint_intensity_index[ch]) {
            core->joint_scale_sel[ch] = bits_get(&core->bits, 3);
            if (core->joint_scale_sel[ch] == 7) {
                core_err("Invalid X96 joint scale factor code book");
                return -DCADEC_EBADDATA;
            }
        }
    }

    // Scale factors for joint subband coding
    for (ch = xch_base; ch < core->x96_nchannels; ch++) {
        // Only if joint subband coding is enabled
        if (core->joint_intensity_index[ch]) {
            // Select codebook
            int sel = core->joint_scale_sel[ch];
            // Get source channel
            int src_ch = core->joint_intensity_index[ch] - 1;
            for (band = core->nsubbands[ch]; band < core->nsubbands[src_ch]; band++) {
                if ((ret = parse_joint_scale(core, sel)) < 0)
                    return ret;
                core->joint_scale_factors[ch][band] = ret;
            }
        }
    }

    // Side information CRC check word
    if (core->crc_present)
        bits_skip(&core->bits, 16);

    return 0;
}

static int parse_x96_coding_header(struct core_decoder *core, bool exss, int xch_base)
{
    int ch, n, ret, header_size = 0, header_pos = core->bits.index;

    if (exss) {
        // Channel set header length
        header_size = bits_get(&core->bits, 7) + 1;

        // Check CRC
        if (core->x96_crc_present && (ret = bits_check_crc(&core->bits, header_pos, header_pos + header_size * 8)) < 0) {
            core_err("Invalid X96 channel set header checksum");
            return ret;
        }
    }

    // High resolution flag
    core->x96_high_res = bits_get1(&core->bits);

    // First encoded subband
    if (core->x96_rev_no < 8) {
        core->x96_subband_start = bits_get(&core->bits, 5);
        if (core->x96_subband_start > 27) {
            core_err("Invalid X96 subband start index (%d)", core->x96_subband_start);
            return -DCADEC_EBADDATA;
        }
    } else {
        core->x96_subband_start = MAX_SUBBANDS;
    }

    // Subband activity count
    for (ch = xch_base; ch < core->x96_nchannels; ch++) {
        core->nsubbands[ch] = bits_get(&core->bits, 6) + 1;
        if (core->nsubbands[ch] < MAX_SUBBANDS) {
            core_err("Invalid X96 subband activity count (%d)", core->nsubbands[ch]);
            return -DCADEC_EBADDATA;
        }
    }

    // Joint intensity coding index
    for (ch = xch_base; ch < core->x96_nchannels; ch++) {
        if ((n = bits_get(&core->bits, 3)) && xch_base)
            n += xch_base - 1;
        if (n > core->x96_nchannels) {
            core_err("Invalid X96 joint intensity coding index");
            return -DCADEC_EBADDATA;
        }
        core->joint_intensity_index[ch] = n;
    }

    // Scale factor code book
    for (ch = xch_base; ch < core->x96_nchannels; ch++) {
        core->scale_factor_sel[ch] = bits_get(&core->bits, 3);
        if (core->scale_factor_sel[ch] >= 6) {
            core_err("Invalid X96 scale factor code book");
            return -DCADEC_EBADDATA;
        }
    }

    // Bit allocation quantizer select
    for (ch = xch_base; ch < core->x96_nchannels; ch++)
        core->bit_allocation_sel[ch] = bits_get(&core->bits, 3);

    // Quantization index codebook select
    for (n = 0; n < 6 + 4 * core->x96_high_res; n++)
        for (ch = xch_base; ch < core->x96_nchannels; ch++)
            core->quant_index_sel[ch][n] = bits_get(&core->bits, quant_index_sel_nbits[n]);

    if (exss) {
        // Reserved
        // Byte align
        // CRC16 of channel set header
        if ((ret = bits_seek(&core->bits, header_pos + header_size * 8)) < 0) {
            core_err("Read past end of X96 channel set header");
            return ret;
        }
    } else {
        if (core->crc_present)
            bits_skip(&core->bits, 16);
    }

    return 0;
}

static int parse_x96_frame_data(struct core_decoder *core, bool exss, int xch_base)
{
    int ret;
    if ((ret = parse_x96_coding_header(core, exss, xch_base)) < 0)
        return ret;

    int sub_pos = 0;
    for (int sf = 0; sf < core->nsubframes; sf++) {
        if ((ret = parse_x96_subframe_header(core, xch_base)) < 0)
            return ret;
        if ((ret = parse_x96_subframe_audio(core, sf, xch_base, &sub_pos)) < 0)
            return ret;
    }

    for (int ch = xch_base; ch < core->x96_nchannels; ch++) {
        // Number of active subbands for this channel
        int nsubbands = core->nsubbands[ch];
        if (core->joint_intensity_index[ch])
            nsubbands = DCA_MAX(nsubbands, core->nsubbands[core->joint_intensity_index[ch] - 1]);

        // Update history for ADPCM
        // Clear inactive subbands
        for (int band = 0; band < MAX_SUBBANDS_X96; band++) {
            int *samples = core->x96_subband_samples[ch][band] - NUM_ADPCM_COEFFS;
            if (band >= core->x96_subband_start && band < nsubbands)
                memcpy(samples, samples + core->npcmblocks, NUM_ADPCM_COEFFS * sizeof(int));
            else
                memset(samples, 0, (NUM_ADPCM_COEFFS + core->npcmblocks) * sizeof(int));
        }
    }

    return 0;
}

static int parse_x96_frame(struct core_decoder *core)
{
    // Revision number
    core->x96_rev_no = bits_get(&core->bits, 4);
    if (core->x96_rev_no < 1 || core->x96_rev_no > 8) {
        core_err_once("Unsupported X96 revision (%d)", core->x96_rev_no);
        return -DCADEC_ENOSUP;
    }

    core->x96_crc_present = false;
    core->x96_nchannels = core->nchannels;

    int ret;
    if ((ret = alloc_x96_sample_buffer(core)) < 0)
        return ret;

    if ((ret = parse_x96_frame_data(core, false, 0)) < 0)
        return ret;

    // Seek to the end of core frame
    return bits_seek(&core->bits, core->frame_size * 8);
}

static int parse_x96_frame_exss(struct core_decoder *core)
{
    int     x96_frame_size[MAX_EXSS_CHSETS];
    int     x96_nchannels[MAX_EXSS_CHSETS];

    int header_pos = core->bits.index;

    // X96 sync word
    if (bits_get(&core->bits, 32) != SYNC_WORD_X96) {
        core_err("Invalid X96 sync word");
        return -DCADEC_ENOSYNC;
    }

    // X96 frame header length
    int header_size = bits_get(&core->bits, 6) + 1;

    // Check X96 frame header CRC
    int ret;
    if ((ret = bits_check_crc(&core->bits, header_pos + 32, header_pos + header_size * 8)) < 0) {
        core_err("Invalid X96 frame header checksum");
        return ret;
    }

    // Revision number
    core->x96_rev_no = bits_get(&core->bits, 4);
    if (core->x96_rev_no < 1 || core->x96_rev_no > 8) {
        core_err_once("Unsupported X96 revision (%d)", core->x96_rev_no);
        return -DCADEC_ENOSUP;
    }

    // CRC presence flag for channel set header
    core->x96_crc_present = bits_get1(&core->bits);

    // Number of channel sets
    int x96_nchsets = bits_get(&core->bits, 2) + 1;

    // Channel set data byte size
    for (int i = 0; i < x96_nchsets; i++)
        x96_frame_size[i] = bits_get(&core->bits, 12) + 1;

    // Number of channels in channel set
    for (int i = 0; i < x96_nchsets; i++)
        x96_nchannels[i] = bits_get(&core->bits, 3) + 1;

    // Reserved
    // Byte align
    // CRC16 of X96 frame header
    if ((ret = bits_seek(&core->bits, header_pos + header_size * 8)) < 0) {
        core_err("Read past end of X96 frame header");
        return ret;
    }

    if ((ret = alloc_x96_sample_buffer(core)) < 0)
        return ret;

    core->x96_nchannels = 0;

    // Channel set data
    int x96_base_ch = 0;
    for (int i = 0; i < x96_nchsets; i++) {
        header_pos = core->bits.index;

        if (x96_base_ch + x96_nchannels[i] <= core->nchannels) {
            core->x96_nchannels = x96_base_ch + x96_nchannels[i];
            if ((ret = parse_x96_frame_data(core, true, x96_base_ch)) < 0)
                return ret;
        }

        x96_base_ch += x96_nchannels[i];

        if ((ret = bits_seek(&core->bits, header_pos + x96_frame_size[i] * 8)) < 0) {
            core_err("Read past end of X96 channel set");
            return ret;
        }
    }

    return 0;
}

static int parse_aux_data(struct core_decoder *core)
{
    // Auxiliary data byte count (can't be trusted)
    bits_skip(&core->bits, 6);

    // 4-byte align
    int aux_pos = bits_align4(&core->bits);

    // Auxiliary data sync word
    uint32_t sync = bits_get(&core->bits, 32);
    if (sync != SYNC_WORD_REV1AUX) {
        core_err("Invalid auxiliary data sync word (%#x)", sync);
        return -DCADEC_ENOSYNC;
    }

    // Auxiliary decode time stamp flag
    if (bits_get1(&core->bits)) {
        bits_skip(&core->bits,  3); // 4-bit align
        bits_skip(&core->bits,  8); // MSB
        bits_skip(&core->bits,  4); // Marker
        bits_skip(&core->bits, 28); // LSB
        bits_skip(&core->bits,  4); // Marker
    }

    // Auxiliary dynamic downmix flag
    core->prim_dmix_embedded = bits_get1(&core->bits);

    if (core->prim_dmix_embedded) {
        // Auxiliary primary channel downmix type
        core->prim_dmix_type = bits_get(&core->bits, 3);
        if (core->prim_dmix_type >= DMIX_TYPE_COUNT) {
            core_err("Invalid primary channel set downmix type");
            return -DCADEC_EBADDATA;
        }

        // Size of downmix coefficients matrix
        int m = dmix_primary_nch[core->prim_dmix_type];
        int n = audio_mode_nch[core->audio_mode] + !!core->lfe_present;

        // Dynamic downmix code coefficients
        int *coeff_ptr = core->prim_dmix_coeff;
        for (int i = 0; i < m * n; i++) {
            int code = bits_get(&core->bits, 9);
            int sign = (code >> 8) - 1;
            if (code &= 0xff) {
                unsigned int index = code - 1;
                if (index >= dca_countof(dmix_table)) {
                    core_err("Invalid downmix coefficient index");
                    return -DCADEC_EBADDATA;
                }
                *coeff_ptr++ = (dmix_table[index] ^ sign) - sign;
            } else {
                *coeff_ptr++ = 0;
            }
        }
    }

    // Byte align
    bits_align1(&core->bits);

    // CRC16 of auxiliary data
    bits_skip(&core->bits, 16);

    // Check CRC
    int ret;
    if ((ret = bits_check_crc(&core->bits, aux_pos + 32, core->bits.index)) < 0)
        core_err("Invalid auxiliary data checksum");
    return ret;
}

#define CHECK_SYNC(pos, msg) \
    if (!pos) { \
        if (flags & DCADEC_FLAG_STRICT) { \
            core_err(msg); \
            return -DCADEC_ENOSYNC; \
        } \
        core_warn_once(msg); \
        status = DCADEC_WCOREEXTFAILED; \
    }

static int parse_optional_info(struct core_decoder *core, int flags)
{
    int status = 0;

    // Time code stamp
    if (core->ts_present)
        bits_skip(&core->bits, 32);

    // Auxiliary data
    if (core->aux_present && (flags & DCADEC_FLAG_KEEP_DMIX_2CH)) {
        int ret;
        if ((ret = parse_aux_data(core)) < 0) {
            if (flags & DCADEC_FLAG_STRICT)
                return ret;
            status = DCADEC_WCOREAUXFAILED;
            core->prim_dmix_embedded = false;
        }
    } else {
        core->prim_dmix_embedded = false;
    }

    // Core extensions
    if (core->ext_audio_present && !(flags & DCADEC_FLAG_CORE_ONLY)) {
        int sync_pos = DCA_MIN(core->frame_size / 4, core->bits.total / 32) - 1;
        int last_pos = core->bits.index / 32;

        // Search for extension sync words aligned on 4-byte boundary
        switch (core->ext_audio_type) {
        case EXT_AUDIO_XCH:
            if (flags & DCADEC_FLAG_KEEP_DMIX_MASK)
                break;

            // The distance between XCH sync word and end of the core frame
            // must be equal to XCH frame size. Off by one error is allowed for
            // compatibility with legacy bitstreams. Minimum XCH frame size is
            // 96 bytes. AMODE and PCHS are further checked to reduce
            // probability of alias sync detection.
            for (; sync_pos >= last_pos; sync_pos--) {
                if (core->bits.data[sync_pos] == DCA_32BE_C(SYNC_WORD_XCH)) {
                    core->bits.index = (sync_pos + 1) * 32;
                    int frame_size = bits_get(&core->bits, 10) + 1;
                    int dist = core->frame_size - sync_pos * 4;
                    if (frame_size >= 96
                        && (frame_size == dist || frame_size - 1 == dist)
                        && bits_get(&core->bits, 7) == 0x08) {
                        core->xch_pos = core->bits.index;
                        break;
                    }
                }
            }

            CHECK_SYNC(core->xch_pos, "XCH sync word not found")
            break;

        case EXT_AUDIO_X96:
            // The distance between X96 sync word and end of the core frame
            // must be equal to X96 frame size. Minimum X96 frame size is 96
            // bytes.
            for (; sync_pos >= last_pos; sync_pos--) {
                if (core->bits.data[sync_pos] == DCA_32BE_C(SYNC_WORD_X96)) {
                    core->bits.index = (sync_pos + 1) * 32;
                    int frame_size = bits_get(&core->bits, 12) + 1;
                    int dist = core->frame_size - sync_pos * 4;
                    if (frame_size >= 96 && frame_size == dist) {
                        core->x96_pos = core->bits.index;
                        break;
                    }
                }
            }

            CHECK_SYNC(core->x96_pos, "X96 sync word not found")
            break;

        case EXT_AUDIO_XXCH:
            if (flags & DCADEC_FLAG_KEEP_DMIX_MASK)
                break;

            // XXCH frame header CRC must be valid. Minimum XXCH frame header
            // size is 11 bytes.
            for (; sync_pos >= last_pos; sync_pos--) {
                if (core->bits.data[sync_pos] == DCA_32BE_C(SYNC_WORD_XXCH)) {
                    core->bits.index = (sync_pos + 1) * 32;
                    int hdr_size = bits_get(&core->bits, 6) + 1;
                    if (hdr_size >= 11 &&
                        !bits_check_crc(&core->bits, (sync_pos + 1) * 32,
                                        sync_pos * 32 + hdr_size * 8)) {
                        core->xxch_pos = sync_pos * 32;
                        break;
                    }
                }
            }

            CHECK_SYNC(core->xxch_pos, "XXCH sync word not found")
            break;

        default:
            core_warn_once("Stream with unknown extended audio type (%d)",
                           core->ext_audio_type);
            break;
        }
    }

    return status;
}

int core_parse(struct core_decoder *core, uint8_t *data, int size,
               int flags, struct exss_asset *asset)
{
    core->ext_audio_mask = 0;
    core->xch_pos = core->xxch_pos = core->x96_pos = 0;

    if (asset) {
        bits_init(&core->bits, data + asset->core_offset, asset->core_size);
        if (bits_get(&core->bits, 32) != SYNC_WORD_CORE_EXSS)
            return -DCADEC_ENOSYNC;
    } else {
        bits_init(&core->bits, data, size);
        bits_skip(&core->bits, 32);
    }

    int status = 0, ret;
    if ((ret = parse_frame_header(core)) < 0)
        return ret;
    if ((ret = alloc_sample_buffer(core)) < 0)
        return ret;
    if ((ret = parse_frame_data(core, HEADER_CORE, 0)) < 0)
        return ret;
    if ((ret = parse_optional_info(core, flags)) < 0)
        return ret;
    if (ret > 0)
        status = ret;

    // Workaround for DTS in WAV
    if (!asset && core->frame_size > size && core->frame_size < size + 4) {
        core_warn_once("Stream with excessive core frame size");
        core->frame_size = size;
    }

    if ((ret = bits_seek(&core->bits, core->frame_size * 8)) < 0)
        return ret;
    return status;
}

int core_parse_exss(struct core_decoder *core, uint8_t *data,
                    int flags, struct exss_asset *asset)
{
    struct bitstream temp = core->bits;
    int exss_mask = asset ? asset->extension_mask : 0;
    int status = 0, ret = 0, ext = 0;

    // Parse (X)XCH unless downmixing
    if (!(flags & DCADEC_FLAG_KEEP_DMIX_MASK)) {
        if (exss_mask & EXSS_XXCH) {
            bits_init(&core->bits, data + asset->xxch_offset, asset->xxch_size);
            ret = parse_xxch_frame(core);
            ext = EXSS_XXCH;
        } else if (core->xxch_pos) {
            core->bits.index = core->xxch_pos;
            ret = parse_xxch_frame(core);
            ext = CSS_XXCH;
        } else if (core->xch_pos) {
            core->bits.index = core->xch_pos;
            ret = parse_xch_frame(core);
            ext = CSS_XCH;
        }

        // Revert to primary channel set in case (X)XCH parsing fails
        if (ret < 0) {
            if (flags & DCADEC_FLAG_STRICT)
                return ret;
            status = DCADEC_WCOREEXTFAILED;
            core->nchannels = audio_mode_nch[core->audio_mode];
            core->ch_mask = audio_mode_ch_mask[core->audio_mode];
            if (core->lfe_present)
                core->ch_mask |= SPEAKER_MASK_LFE1;
        } else {
            core->ext_audio_mask |= ext;
        }
    }

    // Parse XBR
    if (exss_mask & EXSS_XBR) {
        bits_init(&core->bits, data + asset->xbr_offset, asset->xbr_size);
        if ((ret = parse_xbr_frame(core)) < 0) {
            if (flags & DCADEC_FLAG_STRICT)
                return ret;
            status = DCADEC_WCOREEXTFAILED;
        } else {
            core->ext_audio_mask |= EXSS_XBR;
        }
    }

    // Parse X96
    if (exss_mask & EXSS_X96) {
        bits_init(&core->bits, data + asset->x96_offset, asset->x96_size);
        if ((ret = parse_x96_frame_exss(core)) < 0) {
            if (flags & DCADEC_FLAG_STRICT)
                return ret;
            status = DCADEC_WCOREEXTFAILED;
        } else {
            core->ext_audio_mask |= EXSS_X96;
        }
    } else if (core->x96_pos) {
        core->bits = temp;
        core->bits.index = core->x96_pos;
        if ((ret = parse_x96_frame(core)) < 0) {
            if (flags & DCADEC_FLAG_STRICT)
                return ret;
            status = DCADEC_WCOREEXTFAILED;
        } else {
            core->ext_audio_mask |= CSS_X96;
        }
    }

    return status;
}

void core_clear(struct core_decoder *core)
{
    if (core) {
        if (core->subband_buffer) {
            erase_adpcm_history(core);
            memset(core->lfe_samples, 0, MAX_LFE_HISTORY * sizeof(int));
        }
        if (core->x96_subband_buffer)
            erase_x96_adpcm_history(core);
        for (int ch = 0; ch < MAX_CHANNELS; ch++)
            interpolator_clear(core->subband_dsp[ch]);
        core->output_history_lfe = 0;
    }
}

struct dcadec_core_info *core_get_info(struct core_decoder *core)
{
    struct dcadec_core_info *info = ta_znew(NULL, struct dcadec_core_info);
    if (!info)
        return NULL;
    info->nchannels = audio_mode_nch[core->audio_mode];
    info->audio_mode = core->audio_mode;
    info->lfe_present = core->lfe_present;
    info->sample_rate = core->sample_rate;
    info->source_pcm_res = core->source_pcm_res;
    info->es_format = core->es_format;
    info->bit_rate = core->bit_rate;
    info->npcmblocks = core->npcmblocks;
    info->ext_audio_present = core->ext_audio_present;
    info->ext_audio_type = core->ext_audio_type;
    return info;
}

static int make_spkr_pair_mask(int mask1)
{
    int mask2 = 0;

#define MAP(m1, m2)  if ((mask1 & (m1)) == (m1))    mask2 |= (m2);
    MAP(SPEAKER_MASK_C,                         SPEAKER_PAIR_C)
    MAP(SPEAKER_MASK_L   | SPEAKER_MASK_R,      SPEAKER_PAIR_LR)
    MAP(SPEAKER_MASK_Ls  | SPEAKER_MASK_Rs,     SPEAKER_PAIR_LsRs)
    MAP(SPEAKER_MASK_LFE1,                      SPEAKER_PAIR_LFE1)
    MAP(SPEAKER_MASK_Cs,                        SPEAKER_PAIR_Cs)
    MAP(SPEAKER_MASK_Lh  | SPEAKER_MASK_Rh,     SPEAKER_PAIR_LhRh)
    MAP(SPEAKER_MASK_Lsr | SPEAKER_MASK_Rsr,    SPEAKER_PAIR_LsrRsr)
    MAP(SPEAKER_MASK_Ch,                        SPEAKER_PAIR_Ch)
    MAP(SPEAKER_MASK_Oh,                        SPEAKER_PAIR_Oh)
    MAP(SPEAKER_MASK_Lc  | SPEAKER_MASK_Rc,     SPEAKER_PAIR_LcRc)
    MAP(SPEAKER_MASK_Lw  | SPEAKER_MASK_Rw,     SPEAKER_PAIR_LwRw)
    MAP(SPEAKER_MASK_Lss | SPEAKER_MASK_Rss,    SPEAKER_PAIR_LssRss)
    MAP(SPEAKER_MASK_LFE2,                      SPEAKER_PAIR_LFE2)
    MAP(SPEAKER_MASK_Lhs | SPEAKER_MASK_Rhs,    SPEAKER_PAIR_LhsRhs)
    MAP(SPEAKER_MASK_Chr,                       SPEAKER_PAIR_Chr)
    MAP(SPEAKER_MASK_Lhr | SPEAKER_MASK_Rhr,    SPEAKER_PAIR_LhrRhr)
#undef MAP

    return mask2;
}

struct dcadec_exss_info *core_get_info_exss(struct core_decoder *core)
{
    struct dcadec_exss_info *info = ta_znew(NULL, struct dcadec_exss_info);
    if (!info)
        return NULL;

    info->nchannels = core->nchannels + !!core->lfe_present;
    info->sample_rate = core->sample_rate << !!(core->ext_audio_mask & CSS_X96);
    info->bits_per_sample = core->source_pcm_res;

    if (core->ext_audio_mask & (CSS_XXCH | CSS_XCH))
        info->profile = DCADEC_PROFILE_DS_ES;
    else if (core->ext_audio_mask & CSS_X96)
        info->profile = DCADEC_PROFILE_DS_96_24;
    else
        info->profile = DCADEC_PROFILE_DS;

    info->embedded_6ch = !!(core->ext_audio_mask & (CSS_XXCH | CSS_XCH));
    info->spkr_mask = make_spkr_pair_mask(core->ch_mask);

    if (core->audio_mode == AMODE_STEREO_TOTAL)
        info->matrix_encoding = DCADEC_MATRIX_ENCODING_SURROUND;

    return info;
}

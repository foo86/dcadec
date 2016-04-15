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
#define MAX_CHANNELS_CHSET      8

#define MAX_EXSS_CHSETS         4

#define core_err(...)   dca_log(core, ERROR, __VA_ARGS__)
#define core_warn(...)  dca_log(core, WARNING, __VA_ARGS__)

#define core_err_once(...)     dca_log_once(core, ERROR, __VA_ARGS__)
#define core_warn_once(...)    dca_log_once(core, WARNING, __VA_ARGS__)

struct exss_asset;

struct core_decoder {
    struct dcadec_context   *ctx;   ///< Parent context
    struct bitstream        bits;   ///< Bitstream reader

    // Bit stream header
    bool    crc_present;        ///< CRC present flag
    int     npcmblocks;         ///< Number of PCM sample blocks
    int     frame_size;         ///< Primary frame byte size
    int     audio_mode;         ///< Audio channel arrangement
    int     sample_rate;        ///< Core audio sampling frequency
    int     bit_rate;           ///< Transmission bit rate
    bool    drc_present;        ///< Embedded dynamic range flag
    bool    ts_present;         ///< Embedded time stamp flag
    bool    aux_present;        ///< Auxiliary data flag
    int     ext_audio_type;     ///< Extension audio descriptor flag
    bool    ext_audio_present;  ///< Extended coding flag
    bool    sync_ssf;           ///< Audio sync word insertion flag
    int     lfe_present;        ///< Low frequency effects flag
    bool    predictor_history;  ///< Predictor history flag switch
    bool    filter_perfect;     ///< Multirate interpolator switch
    int     source_pcm_res;     ///< Source PCM resolution
    bool    es_format;          ///< Extended surround (ES) mastering flag
    bool    sumdiff_front;      ///< Front sum/difference flag
    bool    sumdiff_surround;   ///< Surround sum/difference flag

    // Primary audio coding header
    int         nsubframes;     ///< Number of subframes
    int         nchannels;      ///< Number of primary audio channels (incl. extension channels)
    int         ch_mask;        ///< Speaker layout mask (incl. LFE and extension channels)
    int8_t      nsubbands[MAX_CHANNELS];                ///< Subband activity count
    int8_t      subband_vq_start[MAX_CHANNELS];         ///< High frequency VQ start subband
    int8_t      joint_intensity_index[MAX_CHANNELS];    ///< Joint intensity coding index
    int8_t      transition_mode_sel[MAX_CHANNELS];      ///< Transient mode code book
    int8_t      scale_factor_sel[MAX_CHANNELS];         ///< Scale factor code book
    int8_t      bit_allocation_sel[MAX_CHANNELS];       ///< Bit allocation quantizer select
    int8_t      quant_index_sel[MAX_CHANNELS][NUM_CODE_BOOKS];  ///< Quantization index codebook select
    int32_t     scale_factor_adj[MAX_CHANNELS][NUM_CODE_BOOKS]; ///< Scale factor adjustment

    // Primary audio coding side information
    int8_t      nsubsubframes[MAX_SUBFRAMES];   ///< Subsubframe count for each subframe
    int8_t      prediction_mode[MAX_CHANNELS][MAX_SUBBANDS_X96];            ///< Prediction mode
    int16_t     prediction_vq_index[MAX_CHANNELS][MAX_SUBBANDS_X96];        ///< Prediction coefficients VQ address
    int8_t      bit_allocation[MAX_CHANNELS][MAX_SUBBANDS_X96];             ///< Bit allocation index
    int8_t      transition_mode[MAX_SUBFRAMES][MAX_CHANNELS][MAX_SUBBANDS]; ///< Transition mode
    union {
        int32_t     scale_factors[MAX_CHANNELS][MAX_SUBBANDS][2];               ///< Scale factors (2x for transients)
        int32_t     x96_scale_factors[MAX_CHANNELS][MAX_SUBBANDS_X96];          ///< X96 scale factors
    };
    int8_t      joint_scale_sel[MAX_CHANNELS];                              ///< Joint subband codebook select
    int32_t     joint_scale_factors[MAX_CHANNELS][MAX_SUBBANDS_X96];        ///< Scale factors for joint subband coding

    // Auxiliary data
    bool    prim_dmix_embedded; ///< Auxiliary dynamic downmix flag
    int     prim_dmix_type;     ///< Auxiliary primary channel downmix type
    int     prim_dmix_coeff[MAX_CHANNELS_DMIX * MAX_CHANNELS_CORE]; ///< Dynamic downmix code coefficients

    // Core extensions
    int     ext_audio_mask;     ///< Bit mask of fully decoded core extensions

    // XCH extension data
    int     xch_pos;    ///< Bit position of XCH frame in core substream

    // XXCH extension data
    bool    xxch_crc_present;       ///< CRC presence flag for XXCH channel set header
    int     xxch_mask_nbits;        ///< Number of bits for loudspeaker mask
    int     xxch_core_mask;         ///< Core loudspeaker activity mask
    int     xxch_spkr_mask;         ///< Loudspeaker layout mask
    bool    xxch_dmix_embedded;     ///< Downmix already performed by encoder
    int     xxch_dmix_scale_inv;    ///< Downmix scale factor
    int     xxch_dmix_mask[MAX_CHANNELS_XXCH];  ///< Downmix channel mapping mask
    int     xxch_dmix_coeff[MAX_CHANNELS_XXCH * MAX_CHANNELS_CORE];     ///< Downmix coefficients
    int     xxch_pos;   ///< Bit position of XXCH frame in core substream

    // X96 extension data
    int     x96_rev_no;         ///< X96 revision number
    bool    x96_crc_present;    ///< CRC presence flag for X96 channel set header
    int     x96_nchannels;      ///< Number of primary channels in X96 extension
    bool    x96_high_res;       ///< X96 high resolution flag
    int     x96_subband_start;  ///< First encoded subband in X96 extension
    int     x96_rand;           ///< Random seed for generating samples for unallocated X96 subbands
    int     x96_pos;            ///< Bit position of X96 frame in core substream

    int     *x96_subband_buffer;    ///< X96 subband sample buffer base
    int     *x96_subband_samples[MAX_CHANNELS][MAX_SUBBANDS_X96];   ///< X96 subband samples

    // Core subband buffer and filter banks
    int                 *subband_buffer;    ///< Subband sample buffer base
    int                 *subband_samples[MAX_CHANNELS][MAX_SUBBANDS];   ///< Subband samples
    struct interpolator *subband_dsp[MAX_CHANNELS]; ///< Filter banks
    struct idct_context *subband_dsp_idct[2];       ///< IDCT context
    int                 *lfe_samples;   ///< Buffer for decimated LFE samples

    // PCM output data
    int     *output_buffer;                 ///< PCM output buffer base
    int     *output_samples[SPEAKER_COUNT]; ///< PCM output speaker map
    int     output_history_lfe;             ///< LFE PCM history for X96 filter

    int     npcmsamples;    ///< Number of PCM samples per channel
    int     output_rate;    ///< Output sample rate (1x or 2x header rate)

    int     filter_flags;   ///< Previous filtering flags for detecting changes
};

int core_parse(struct core_decoder *core, uint8_t *data, int size,
               int flags, struct exss_asset *asset);
int core_parse_exss(struct core_decoder *core, uint8_t *data,
                    int flags, struct exss_asset *asset);
int core_filter(struct core_decoder *core, int flags);
void core_clear(struct core_decoder *core) __attribute__((cold));
struct dcadec_core_info *core_get_info(struct core_decoder *core) __attribute__((cold));
struct dcadec_exss_info *core_get_info_exss(struct core_decoder *core) __attribute__((cold));

#endif

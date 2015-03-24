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
#include "core_decoder.h"
#include "exss_parser.h"
#include "xll_decoder.h"
#include "fixed_math.h"
#include "dmix_tables.h"

#define DCADEC_PACKET_CORE  0x01
#define DCADEC_PACKET_EXSS  0x02
#define DCADEC_PACKET_XLL   0x04

struct dcadec_context {
    int flags;
    int packet;

    struct core_decoder *core;
    struct exss_parser *exss;
    struct xll_decoder *xll;

    unsigned int ncoreframes;

    int nframesamples;
    int sample_rate;
    int bits_per_sample;
    int profile;
    int channel_mask;
    int *samples[SPEAKER_COUNT];
};

static const uint8_t dca2wav_norm[] = {
    WAVESPKR_FC,  WAVESPKR_FL,  WAVESPKR_FR,  WAVESPKR_SL,
    WAVESPKR_SR,  WAVESPKR_LFE, WAVESPKR_BC,  WAVESPKR_BL,
    WAVESPKR_BR,  WAVESPKR_SL,  WAVESPKR_SR,  WAVESPKR_FLC,
    WAVESPKR_FRC, WAVESPKR_TFL, WAVESPKR_TFC, WAVESPKR_TFR,
    WAVESPKR_LFE, WAVESPKR_FLC, WAVESPKR_FRC, WAVESPKR_TC,
    WAVESPKR_TFL, WAVESPKR_TFR, WAVESPKR_TBC, WAVESPKR_TBL,
    WAVESPKR_TBR, WAVESPKR_BC,  WAVESPKR_BL,  WAVESPKR_BR
};

static const uint8_t dca2wav_wide[] = {
    WAVESPKR_FC,  WAVESPKR_FL,  WAVESPKR_FR,  WAVESPKR_BL,
    WAVESPKR_BR,  WAVESPKR_LFE, WAVESPKR_BC,  WAVESPKR_BL,
    WAVESPKR_BR,  WAVESPKR_SL,  WAVESPKR_SR,  WAVESPKR_FLC,
    WAVESPKR_FRC, WAVESPKR_TFL, WAVESPKR_TFC, WAVESPKR_TFR,
    WAVESPKR_LFE, WAVESPKR_SL,  WAVESPKR_SR , WAVESPKR_TC,
    WAVESPKR_TFL, WAVESPKR_TFR, WAVESPKR_TBC, WAVESPKR_TBL,
    WAVESPKR_TBR, WAVESPKR_BC,  WAVESPKR_BL,  WAVESPKR_BR
};

#define DCADEC_LAYOUT_7POINT0_WIDE  \
    (SPEAKER_MASK_C  | SPEAKER_MASK_L  | SPEAKER_MASK_R |   \
     SPEAKER_MASK_Ls | SPEAKER_MASK_Rs |                    \
     SPEAKER_MASK_Lw | SPEAKER_MASK_Rw)

#define DCADEC_LAYOUT_7POINT1_WIDE  \
    (DCADEC_LAYOUT_7POINT0_WIDE | SPEAKER_MASK_LFE1)

static int reorder_samples(struct dcadec_context *dca, int **dca_samples, int dca_mask)
{
    int nchannels = 0;

    if (dca->flags & DCADEC_FLAG_NATIVE_LAYOUT) {
        for (int dca_ch = 0; dca_ch < SPEAKER_COUNT; dca_ch++) {
            if (dca_mask & (1 << dca_ch)) {
                if (!dca_samples[dca_ch])
                    return -DCADEC_EINVAL;
                dca->samples[nchannels++] = dca_samples[dca_ch];
            }
        }
        dca->channel_mask = dca_mask;
    } else {
        int wav_mask = 0;
        int *wav_samples[WAVESPKR_COUNT] = { NULL };
        const uint8_t *dca2wav;
        if (dca_mask == DCADEC_LAYOUT_7POINT0_WIDE ||
            dca_mask == DCADEC_LAYOUT_7POINT1_WIDE)
            dca2wav = dca2wav_wide;
        else
            dca2wav = dca2wav_norm;
        for (size_t dca_ch = 0; dca_ch < sizeof(dca2wav_norm); dca_ch++) {
            if (dca_mask & (1 << dca_ch)) {
                if (!dca_samples[dca_ch])
                    return -DCADEC_EINVAL;
                int wav_ch = dca2wav[dca_ch];
                if (!wav_samples[wav_ch]) {
                    wav_samples[wav_ch] = dca_samples[dca_ch];
                    wav_mask |= 1 << wav_ch;
                }
            }
        }
        for (int wav_ch = 0; wav_ch < WAVESPKR_COUNT; wav_ch++) {
            if (wav_mask & (1 << wav_ch)) {
                dca->samples[nchannels++] = wav_samples[wav_ch];
            }
        }
        dca->channel_mask = wav_mask;
    }

    return nchannels;
}

static int filter_core_frame(struct dcadec_context *dca)
{
    struct core_decoder *core = dca->core;

    dca->ncoreframes = 0;

    // Filter core frame
    int ret;
    if ((ret = core_filter(core, dca->flags)) < 0)
        return ret;

    // Reorder sample buffer pointers
    if (reorder_samples(dca, core->output_samples, core->ch_mask) <= 0)
        return -DCADEC_EINVAL;

    dca->nframesamples = core->npcmsamples;
    dca->sample_rate = core->output_rate;
    dca->bits_per_sample = core->bits_per_sample;
    if (core->xbr_present || core->xxch_present)
        dca->profile = DCADEC_PROFILE_HD_HRA;
    else if (core->es_format && core->xch_present)
        dca->profile = DCADEC_PROFILE_DS_ES;
    else if (core->x96_present && !(dca->flags & DCADEC_FLAG_CORE_SYNTH_X96))
        dca->profile = DCADEC_PROFILE_DS_96_24;
    else
        dca->profile = DCADEC_PROFILE_DS;
    return 0;
}

static int map_spkr_to_core_ch(struct core_decoder *core, int spkr)
{
    if (core->ch_mask & (1 << spkr))
        return spkr;
    if (spkr == SPEAKER_Lss && (core->ch_mask & SPEAKER_MASK_Ls))
        return SPEAKER_Ls;
    if (spkr == SPEAKER_Rss && (core->ch_mask & SPEAKER_MASK_Rs))
        return SPEAKER_Rs;
    return -1;
}

static struct xll_chset *find_hier_dmix_chset(struct xll_decoder *xll, struct xll_chset *c)
{
    c++;
    while (c != &xll->chset[xll->nchsets]) {
        if (!c->primary_chset
            && c->dmix_embedded
            && c->hier_chset
            && !c->replace_set_index
            && c->ch_mask_enabled)
            return c;
        c++;
    }
    return NULL;
}

static int conv_dmix_scale(int code)
{
    unsigned int index = (code & 0xff) - 1;
    if (index < dca_countof(dmix_table)) {
        int sign = (code >> 8) - 1;
        int coeff = dmix_table[index];
        return (coeff ^ sign) - sign;
    }
    return 0;
}

static int conv_dmix_scale_inv(int code)
{
    unsigned int index = (code & 0xff) - 41;
    if (index < dca_countof(dmix_table_inv)) {
        int sign = (code >> 8) - 1;
        int coeff = dmix_table_inv[index];
        return (coeff ^ sign) - sign;
    }
    return 0;
}

static int undo_down_mix(struct xll_decoder *xll,
                         struct xll_chset *c,
                         struct xll_chset *o,
                         int **samples, int nchannels)
{
    if (c->freq != o->freq)
        return -DCADEC_ENOSUP;

    if (c->pcm_bit_res != o->pcm_bit_res)
        return -DCADEC_ENOSUP;

    int *coeff_ptr = o->dmix_coeff;
    for (int i = 0; i < nchannels; i++) {
        // Get |InvDmixScale|
        int scale_inv = conv_dmix_scale_inv(*coeff_ptr++ | 0x100);
        for (int j = 0; j < o->nchannels; j++) {
            // Multiply by |InvDmixScale| to get UndoDmixScale
            int coeff = mul16(conv_dmix_scale(*coeff_ptr++), scale_inv);
            if (coeff) {
                int *src = o->msb_sample_buffer[j];
                int *dst = samples[i];
                for (int k = 0; k < xll->nframesamples; k++)
                    dst[k] -= mul15(src[k], coeff);
            }
        }
    }

    return 0;
}

static int filter_hd_ma_frame(struct dcadec_context *dca)
{
    struct core_decoder *core = dca->core;
    struct xll_decoder *xll = dca->xll;
    int ret;

    // Select the first (primary) channel set
    struct xll_chset *p = &xll->chset[0];
    if (!p->primary_chset || p->replace_set_index)
        return -DCADEC_ENOSUP;

    // Filter core frame if present
    if (dca->packet & DCADEC_PACKET_CORE) {
        int flags = DCADEC_FLAG_CORE_BIT_EXACT | DCADEC_FLAG_KEEP_DMIX_6CH;
        if (p->freq == 96000 && core->sample_rate == 48000)
            flags |= DCADEC_FLAG_CORE_SYNTH_X96;
        if ((ret = core_filter(core, flags)) < 0)
            return ret;
        // Force lossy downmixed output if this is the first core frame since
        // the last time history was cleared
        if (dca->ncoreframes == 0 && xll->nchsets > 1) {
            for_each_chset(xll, c) {
                xll_clear_band_data(c);
                c->dmix_embedded = false;
            }
        }
        dca->ncoreframes++;
    }

    int nchannels = 0;

    // Process channel sets
    for_each_chset(xll, c) {
        if (c->replace_set_index)
            continue;

        xll_filter_band_data(c);

        // Check for residual encoded channel set
        if (c->residual_encode != (1 << c->nchannels) - 1) {
            if (!(dca->packet & DCADEC_PACKET_CORE))
                return -DCADEC_EINVAL;

            if (c->freq != core->output_rate)
                return -DCADEC_ENOSUP;

            if (xll->nframesamples != core->npcmsamples)
                return -DCADEC_EINVAL;

            // See if this channel set is downmixed and find the source
            // channel set. If downmixed, undo core pre-scaling before
            // combining with residual (residual is not scaled).
            struct xll_chset *o = find_hier_dmix_chset(xll, c);

            // Reduce core bit width and combine with residual
            for (int ch = 0; ch < c->nchannels; ch++) {
                if (c->residual_encode & (1 << ch))
                    continue;

                int spkr = xll_map_ch_to_spkr(c, ch);
                if (spkr < 0)
                    return -DCADEC_EINVAL;

                int core_ch = map_spkr_to_core_ch(core, spkr);
                if (core_ch < 0)
                    return -DCADEC_EINVAL;

                int shift = 24 - c->pcm_bit_res;
                // Account for LSB width
                if (xll->scalable_lsbs)
                    shift += xll_get_lsb_width(c, ch);
                int round = shift > 0 ? 1 << (shift - 1) : 0;

                int *dst = c->msb_sample_buffer[ch];
                int *src = core->output_samples[core_ch];
                if (o) {
                    // Undo embedded core downmix pre-scaling
                    int coeff = o->dmix_coeff[(nchannels + ch) * (o->nchannels + 1)];
                    int scale_inv = conv_dmix_scale_inv(coeff);
                    for (int n = 0; n < xll->nframesamples; n++)
                        dst[n] += clip23((mul16(src[n], scale_inv) + round) >> shift);
                } else {
                    // No downmix scaling
                    for (int n = 0; n < xll->nframesamples; n++)
                        dst[n] += (src[n] + round) >> shift;
                }
            }
        }

        // Assemble MSB and LSB parts after combining with core
        if (xll->scalable_lsbs)
            xll_assemble_msbs_lsbs(c);

        nchannels += c->nchannels;
    }

    int *spkr_map[SPEAKER_COUNT] = { NULL };
    int ch_mask = 0;

    // Fake up channel mask for primary channel set if needed
    if (!p->ch_mask_enabled) {
        if (p->nchannels == 2)
            p->ch_mask = SPEAKER_MASK_L | SPEAKER_MASK_R;
        else
            return -DCADEC_ENOSUP;
    }

    int *samples[256];

    // Build the output speaker map and channel vector for downmix reversal
    nchannels = 0;
    for_each_chset(xll, c) {
        if (c->replace_set_index)
            continue;
        for (int ch = 0; ch < c->nchannels; ch++) {
            int spkr = xll_map_ch_to_spkr(c, ch);
            if (spkr < 0)
                return -DCADEC_EINVAL;
            if (!spkr_map[spkr])
                spkr_map[spkr] = c->msb_sample_buffer[ch];
            samples[nchannels++] = c->msb_sample_buffer[ch];
        }
        ch_mask |= c->ch_mask;
    }

    // Undo embedded hierarchial downmix
    nchannels = 0;
    for_each_chset(xll, c) {
        if (c->replace_set_index)
            continue;
        nchannels += c->nchannels;
        struct xll_chset *o = find_hier_dmix_chset(xll, c);
        if (o)
            if ((ret = undo_down_mix(xll, c, o, samples, nchannels)) < 0)
                return ret;
    }

    // Shift samples to account for storage bit width
    int shift = p->storage_bit_res - p->pcm_bit_res;
    if (shift < 0)
        return -DCADEC_EINVAL;
    if (shift > 0) {
        for (int spkr = 0; spkr < SPEAKER_COUNT; spkr++) {
            if (ch_mask & (1 << spkr)) {
                int *buf = spkr_map[spkr];
                if (!buf)
                    return -DCADEC_EINVAL;
                for (int n = 0; n < xll->nframesamples; n++)
                    buf[n] <<= shift;
            }
        }
    }

    // Reorder sample buffer pointers
    if (reorder_samples(dca, spkr_map, ch_mask) <= 0)
        return -DCADEC_EINVAL;

    dca->nframesamples = xll->nframesamples;
    dca->sample_rate = p->freq;
    dca->bits_per_sample = p->storage_bit_res;
    dca->profile = DCADEC_PROFILE_HD_MA;
    return 0;
}

DCADEC_API int dcadec_context_parse(struct dcadec_context *dca, uint8_t *data, size_t size)
{
    int ret;

    if (!dca || !data || size < 4 || ((uintptr_t)data & 3))
        return -DCADEC_EINVAL;

    dca->packet = 0;

    uint32_t sync;
    sync  = data[0] << 24;
    sync |= data[1] << 16;
    sync |= data[2] <<  8;
    sync |= data[3] <<  0;

    if (sync == SYNC_WORD_CORE) {
        if (!dca->core)
            if (!(dca->core = ta_znew(dca, struct core_decoder)))
                return -DCADEC_ENOMEM;

        if ((ret = core_parse(dca->core, data, size, dca->flags, NULL)) < 0)
            return ret;

        dca->packet |= DCADEC_PACKET_CORE;

        // EXXS data must be aligned on 4-byte boundary by the caller
        size_t frame_size = (dca->core->frame_size + 3) & ~3;
        if (size - 4 > frame_size) {
            data += frame_size;
            size -= frame_size;
            sync  = data[0] << 24;
            sync |= data[1] << 16;
            sync |= data[2] <<  8;
            sync |= data[3] <<  0;
        }
    }

    if (sync == SYNC_WORD_EXSS) {
        if (!dca->exss)
            if (!(dca->exss = ta_znew(dca, struct exss_parser)))
                return -DCADEC_ENOMEM;

        if ((ret = exss_parse(dca->exss, data, size)) < 0)
            goto fail;

        dca->packet |= DCADEC_PACKET_EXSS;

        struct exss_asset *asset = &dca->exss->assets[0];

        if (!(dca->packet & DCADEC_PACKET_CORE) && (asset->extension_mask & EXSS_CORE)) {
            if (!dca->core)
                if (!(dca->core = ta_znew(dca, struct core_decoder)))
                    return -DCADEC_ENOMEM;

            if ((ret = core_parse(dca->core, data, size, dca->flags, asset)) < 0)
                return ret;

            dca->packet |= DCADEC_PACKET_CORE;
        }

        if (!(dca->flags & DCADEC_FLAG_CORE_ONLY)) {
            if ((dca->packet & DCADEC_PACKET_CORE) && (asset->extension_mask & (EXSS_XBR | EXSS_XXCH | EXSS_X96)))
                if ((ret = core_parse_exss(dca->core, data, size, dca->flags, asset)) < 0)
                    goto fail;

            if (asset->extension_mask & EXSS_XLL) {
                if (!dca->xll)
                    if (!(dca->xll = ta_znew(dca, struct xll_decoder)))
                        return -DCADEC_ENOMEM;

                if ((ret = xll_parse(dca->xll, data, size, asset)) < 0)
                    goto fail;

                dca->packet |= DCADEC_PACKET_XLL;
            }
        }
    } else if (!dca->packet) {
        return -DCADEC_ENOSYNC;
    }

    return 0;

fail:
    if (!dca->packet || (dca->flags & DCADEC_FLAG_STRICT))
        return ret;
    return 0;
}

DCADEC_API struct dcadec_core_info *dcadec_context_get_core_info(struct dcadec_context *dca)
{
    if (!dca)
        return NULL;
    if (!(dca->packet & DCADEC_PACKET_CORE))
        return NULL;
    return core_get_info(dca->core);
}

DCADEC_API void dcadec_context_free_core_info(struct dcadec_core_info *info)
{
    ta_free(info);
}

DCADEC_API struct dcadec_exss_info *dcadec_context_get_exss_info(struct dcadec_context *dca)
{
    struct dcadec_exss_info *info = NULL;

    if (!dca)
        return NULL;

    if (dca->packet & DCADEC_PACKET_EXSS) {
        info = ta_znew(NULL, struct dcadec_exss_info);
        if (info) {
            struct exss_asset *asset = &dca->exss->assets[0];
            info->nchannels = asset->nchannels_total;
            info->sample_rate = asset->max_sample_rate;
            info->bits_per_sample = asset->pcm_bit_res;
            if (asset->extension_mask & EXSS_XLL)
                info->profile = DCADEC_PROFILE_HD_MA;
            else if (asset->extension_mask & (EXSS_XBR | EXSS_XXCH | EXSS_X96))
                info->profile = DCADEC_PROFILE_HD_HRA;
            else if (asset->extension_mask & EXSS_LBR)
                info->profile = DCADEC_PROFILE_EXPRESS;
            else
                info->profile = DCADEC_PROFILE_UNKNOWN;
            info->embedded_stereo = asset->embedded_stereo;
            info->embedded_6ch = asset->embedded_6ch;
        }
    } else if (dca->packet & DCADEC_PACKET_CORE) {
        struct core_decoder *core = dca->core;
        if (core->xch_present || core->xxch_present ||
            core->xbr_present || core->x96_present) {
            info = ta_znew(NULL, struct dcadec_exss_info);
            if (info) {
                info->nchannels = core->nchannels + !!core->lfe_present;
                info->sample_rate = core->sample_rate << core->x96_present;
                info->bits_per_sample = core->source_pcm_res;
                if (core->xbr_present || core->xxch_present)
                    info->profile = DCADEC_PROFILE_HD_HRA;
                else if (core->es_format && core->xch_present)
                    info->profile = DCADEC_PROFILE_DS_ES;
                else if (core->x96_present)
                    info->profile = DCADEC_PROFILE_DS_96_24;
                else
                    info->profile = DCADEC_PROFILE_DS;
            }
        }
    }

    return info;
}

DCADEC_API void dcadec_context_free_exss_info(struct dcadec_exss_info *info)
{
    ta_free(info);
}

DCADEC_API int dcadec_context_filter(struct dcadec_context *dca, int ***samples,
                                     int *nsamples, int *channel_mask,
                                     int *sample_rate, int *bits_per_sample,
                                     int *profile)
{
    int ret;

    if (!dca)
        return -DCADEC_EINVAL;

    if (dca->packet & DCADEC_PACKET_XLL) {
        if ((ret = filter_hd_ma_frame(dca)) < 0)
            return ret;
    } else if (dca->packet & DCADEC_PACKET_CORE) {
        if ((ret = filter_core_frame(dca)) < 0)
            return ret;
    } else {
        return -DCADEC_EINVAL;
    }

    if (samples)
        *samples = dca->samples;
    if (nsamples)
        *nsamples = dca->nframesamples;
    if (channel_mask)
        *channel_mask = dca->channel_mask;
    if (sample_rate)
        *sample_rate = dca->sample_rate;
    if (bits_per_sample)
        *bits_per_sample = dca->bits_per_sample;
    if (profile)
        *profile = dca->profile;
    return 0;
}

DCADEC_API void dcadec_context_clear(struct dcadec_context *dca)
{
    if (dca) {
        core_clear(dca->core);
        xll_clear(dca->xll);
        dca->ncoreframes = 0;
    }
}

DCADEC_API struct dcadec_context *dcadec_context_create(int flags)
{
    struct dcadec_context *dca = ta_znew(NULL, struct dcadec_context);
    if (dca)
        dca->flags = flags;
    return dca;
}

DCADEC_API void dcadec_context_destroy(struct dcadec_context *dca)
{
    ta_free(dca);
}

DCADEC_API const char *dcadec_strerror(int errnum)
{
    static const char * const errors[] = {
        "Invalid argument",
        "Invalid bitstream format",
        "CRC check failed",
        "Bitstream navigation error",
        "Synchronization error",
        "Unsupported feature",
        "Memory allocation error",
        "PCM output overflow",
        "I/O error",
        "PCM output parameters changed"
    };

    if (errnum >= 0)
        return "No error";

    unsigned int err = -errnum - 1;
    if (err < dca_countof(errors))
        return errors[err];
    else
        return "Unspecified error";
}

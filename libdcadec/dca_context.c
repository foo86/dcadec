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

#define MAX_PACKET_SIZE     0x104000

#define PACKET_CORE     0x01
#define PACKET_EXSS     0x02
#define PACKET_XLL      0x04

#define PACKET_FILTERED     0x100
#define PACKET_RECOVERY     0x200

#define dca_warn_once(...) \
    dca_format_log(dca, DCADEC_LOG_WARNING | DCADEC_LOG_ONCE, __FILE__, __LINE__, __VA_ARGS__)

struct dcadec_context {
    dcadec_log_cb   log_cb;         ///< Logging callback function
    void            *log_cbarg;     ///< Logging callback argument
    int             log_shown;      ///< Bitmask of logging levels shown once

    int     flags;  ///< Context flags passed to dcadec_context_create()
    int     packet; ///< Packet flags set by dcadec_context_parse()

    struct core_decoder *core;  ///< Core decoder context
    struct exss_parser  *exss;  ///< EXSS parser context
    struct xll_decoder  *xll;   ///< XLL decoder context

    bool    has_residual_encoded;   ///< XLL residual encoded channels present
    bool    core_residual_valid;    ///< Core valid for residual decoding

    int     *dmix_sample_buffer;    ///< Primary channel set downmixing buffer

    int     status;             ///< Filtering status
    int     nframesamples;      ///< Number of PCM samples per channel
    int     sample_rate;        ///< Sample rate in Hz
    int     bits_per_sample;    ///< PCM resolution in bits
    int     profile;            ///< Type of DTS profile decoded
    int     channel_mask;       ///< Channel or speaker mask
    int     *samples[SPEAKER_COUNT];    ///< Sample buffer pointers
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

void dca_format_log(struct dcadec_context *dca, int level,
                    const char *file, int line, const char *fmt, ...)
{
    char buffer[1024];
    va_list ap;

    if (!dca || !dca->log_cb)
        return;

    if (level & DCADEC_LOG_ONCE) {
        level &= ~DCADEC_LOG_ONCE;
        if (dca->log_shown & (1 << level))
            return;
        dca->log_shown |= 1 << level;
    }

    va_start(ap, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, ap);
    va_end(ap);

    dca->log_cb(level, file, line, buffer, dca->log_cbarg);
}

static int reorder_samples(struct dcadec_context *dca, int **dca_samples, int dca_mask)
{
    int nchannels = 0;

    if (dca->flags & DCADEC_FLAG_NATIVE_LAYOUT) {
        for (int dca_ch = 0; dca_ch < SPEAKER_COUNT; dca_ch++) {
            if (dca_mask & (1U << dca_ch)) {
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
        if (dca_mask == SPEAKER_LAYOUT_7POINT0_WIDE ||
            dca_mask == SPEAKER_LAYOUT_7POINT1_WIDE)
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

static bool shift_and_clip__(int *samples, int nsamples, int shift, int bits)
{
    bool clipped = false;

    for (int n = 0; n < nsamples; n++) {
        int s = samples[n] * (1 << shift);
#ifdef __ARM_FEATURE_SAT
        s = clip__(s, bits);
#else
        if ((s + (1 << bits)) & ~((1 << (bits + 1)) - 1)) {
            s = (s >> 31) ^ ((1 << bits) - 1);
            clipped = true;
        }
#endif
        samples[n] = s;
    }

    return clipped;
}

static bool shift_and_clip(struct dcadec_context *dca, int nchannels,
                           int storage_bit_res, int pcm_bit_res)
{
    int shift = storage_bit_res - pcm_bit_res;
    int nsamples = dca->nframesamples;

    if (dca->flags & DCADEC_FLAG_DONT_CLIP) {
        if (shift)
            for (int ch = 0; ch < nchannels; ch++)
                for (int n = 0; n < nsamples; n++)
                    dca->samples[ch][n] *= 1 << shift;
        return false;
    }

    bool clipped = false;
    switch (storage_bit_res) {
    case 24:
        for (int ch = 0; ch < nchannels; ch++)
            clipped |= shift_and_clip__(dca->samples[ch], nsamples, shift, 23);
        break;
    case 16:
        for (int ch = 0; ch < nchannels; ch++)
            clipped |= shift_and_clip__(dca->samples[ch], nsamples, shift, 15);
        break;
    default:
        assert(0);
        break;
    }

    return clipped;
}

static int get_dmix_coeff(int nchannels, int spkr, int ch)
{
    switch (spkr) {
    case SPEAKER_C:
    case SPEAKER_Cs:
        return (nchannels == 1) ? 23170 : 16423;
    case SPEAKER_L:
        return (ch == 0) ? 23170 : 0;
    case SPEAKER_R:
        return (ch == 1) ? 23170 : 0;
    case SPEAKER_Ls:
        return (ch == 0) ? 16423 : 0;
    case SPEAKER_Rs:
        return (ch == 1) ? 16423 : 0;
    default:
        return 0;
    }
}

static int down_mix_prim_chset(struct dcadec_context *dca,
                               bool dmix_embedded, int dmix_type,
                               int *dmix_coeff_cur, int *dmix_coeff_pre,
                               int **samples, int nsamples, int *ch_mask)
{
    // No action if already 2.0
    if (*ch_mask == SPEAKER_LAYOUT_STEREO)
        return 0;

    if (dmix_embedded && dmix_type != DMIX_TYPE_LoRo) {
        dca_warn_once("Unsupported primary channel set downmix type (%d)", dmix_type);
        dmix_embedded = false;
    }

    if (!dmix_embedded) {
        // Remove LFE channel if 2.1
        if (*ch_mask == SPEAKER_LAYOUT_2POINT1) {
            *ch_mask = SPEAKER_LAYOUT_STEREO;
            return 0;
        }

        // Unless both KEEP_DMIX flags are set, perform 2.0 downmix only when
        // custom matrix is present
        if (!(dca->flags & DCADEC_FLAG_KEEP_DMIX_6CH))
            return 0;
    }

    assert(nsamples > 1);

    // Reallocate downmix sample buffer
    if (ta_alloc_fast(dca, &dca->dmix_sample_buffer, 2 * nsamples, sizeof(int)) < 0)
        return -DCADEC_ENOMEM;

    memset(dca->dmix_sample_buffer, 0, 2 * nsamples * sizeof(int));

    int nchannels = dca_popcount(*ch_mask);
    int nsamples_log2 = 31 - dca_clz(nsamples);

    // Perform downmix
    for (int spkr = 0, pos = 0; spkr < SPEAKER_COUNT; spkr++) {
        if (!(*ch_mask & (1U << spkr)))
            continue;

        int *src = samples[spkr];
        int *dst = dca->dmix_sample_buffer;

        for (int ch = 0; ch < 2; ch++) {
            int coeff_cur, coeff_pre;

            // Use custom matrix if present. Otherwise use default matrix that
            // covers all supported core audio channel arrangements.
            if (dmix_embedded) {
                coeff_cur = dmix_coeff_cur[ch * nchannels + pos];
                coeff_pre = dmix_coeff_pre[ch * nchannels + pos];
            } else {
                coeff_cur = coeff_pre = get_dmix_coeff(nchannels, spkr, ch);
            }

            int delta = coeff_cur - coeff_pre;
            if (delta) {
                // Downmix coefficient interpolation
                int ramp = 1 << (nsamples_log2 - 1);
                for (int n = 0; n < nsamples; n++, ramp += delta)
                    dst[n] += mul15(src[n], coeff_pre + (ramp >> nsamples_log2));
            } else if (coeff_cur) {
                for (int n = 0; n < nsamples; n++)
                    dst[n] += mul15(src[n], coeff_cur);
            }

            dst += nsamples;
        }

        pos++;
    }

    samples[SPEAKER_L] = dca->dmix_sample_buffer;
    samples[SPEAKER_R] = dca->dmix_sample_buffer + nsamples;
    *ch_mask = SPEAKER_LAYOUT_STEREO;
    return 1;
}

static int filter_core_frame(struct dcadec_context *dca)
{
    struct core_decoder *core = dca->core;

    // Filter core frame
    int ret;
    if ((ret = core_filter(core, dca->flags)) < 0) {
        dca->core_residual_valid = false;
        return ret;
    }

    dca->core_residual_valid = !!(dca->flags & DCADEC_FLAG_CORE_BIT_EXACT);

    // Downmix core channels to Lo/Ro
    if (dca->flags & DCADEC_FLAG_KEEP_DMIX_2CH) {
        if ((ret = down_mix_prim_chset(dca,
                                       core->prim_dmix_embedded,
                                       core->prim_dmix_type,
                                       core->prim_dmix_coeff,
                                       core->prim_dmix_coeff,
                                       core->output_samples,
                                       core->npcmsamples,
                                       &core->ch_mask)) < 0)
            return ret;
    }

    // Reorder sample buffer pointers
    int nchannels;
    if ((nchannels = reorder_samples(dca, core->output_samples, core->ch_mask)) <= 0)
        return -DCADEC_EINVAL;

    dca->nframesamples = core->npcmsamples;
    dca->sample_rate = core->output_rate;
    dca->bits_per_sample = 24;

    // Set profile
    if (core->ext_audio_mask & (EXSS_XBR | EXSS_XXCH | EXSS_X96))
        dca->profile = DCADEC_PROFILE_HD_HRA;
    else if (core->ext_audio_mask & (CSS_XXCH | CSS_XCH))
        dca->profile = DCADEC_PROFILE_DS_ES;
    else if (core->ext_audio_mask & CSS_X96)
        dca->profile = DCADEC_PROFILE_DS_96_24;
    else
        dca->profile = DCADEC_PROFILE_DS;

    // Perform clipping after Lo/Ro downmix
    if (ret > 0)
        shift_and_clip(dca, nchannels, 24, 24);

    return 0;
}

static int map_spkr_to_core_spkr(struct core_decoder *core, int spkr)
{
    if (core->ch_mask & (1U << spkr))
        return spkr;
    if (spkr == SPEAKER_Lss && (core->ch_mask & SPEAKER_MASK_Ls))
        return SPEAKER_Ls;
    if (spkr == SPEAKER_Rss && (core->ch_mask & SPEAKER_MASK_Rs))
        return SPEAKER_Rs;
    return -1;
}

static bool is_hier_dmix_chset(struct xll_chset *c)
{
    return !c->primary_chset && c->dmix_embedded && c->hier_chset;
}

static struct xll_chset *find_next_hier_dmix_chset(struct xll_chset *c)
{
    struct xll_decoder *xll = c->decoder;

    if (c->hier_chset)
        while (++c < &xll->chset[xll->nchsets])
            if (is_hier_dmix_chset(c))
                return c;

    return NULL;
}

static void prescale_down_mix(struct xll_chset *c, struct xll_chset *o)
{
    int *coeff_ptr = c->dmix_coeff_cur;
    for (int i = 0; i < c->hier_m; i++) {
        int scale = o->dmix_scale_cur[i];
        int scale_inv = o->dmix_scale_inv_cur[i];
        c->dmix_scale_cur[i] = mul15(c->dmix_scale_cur[i], scale);
        c->dmix_scale_inv_cur[i] = mul16(c->dmix_scale_inv_cur[i], scale_inv);
        for (int j = 0; j < c->nchannels; j++) {
            int coeff = mul16(*coeff_ptr, scale_inv);
            *coeff_ptr++ = mul15(coeff, o->dmix_scale_cur[c->hier_m + j]);
        }
    }
}

struct downmix {
    int *samples[XLL_MAX_BANDS][SPEAKER_COUNT];
    int *deci_history[SPEAKER_COUNT];
};

static void undo_down_mix(struct xll_chset *c, struct downmix *dmix, int band)
{
    struct xll_decoder *xll = c->decoder;
    struct xll_band *b = &c->bands[band];
    int nsamples = xll->nframesamples;
    int nsamples_log2 = xll->nframesamples_log2;

    if (!b->dmix_embedded)
        return;

    for (int i = 0; i < c->hier_m; i++) {
        for (int j = 0; j < c->nchannels; j++) {
            int coeff_cur = c->dmix_coeff_cur[i * c->nchannels + j];
            int coeff_pre = c->dmix_coeff_pre[i * c->nchannels + j];
            int delta = coeff_cur - coeff_pre;

            // Undo downmix of channel samples
            int *src = b->msb_sample_buffer[j];
            int *dst = dmix->samples[band][i];

            if (delta) {
                // Downmix coefficient interpolation
                int ramp = 1 << (nsamples_log2 - 1);
                for (int k = 0; k < nsamples; k++, ramp += delta)
                    dst[k] -= mul15(src[k], coeff_pre + (ramp >> nsamples_log2));
            } else if (coeff_cur) {
                for (int k = 0; k < nsamples; k++)
                    dst[k] -= mul15(src[k], coeff_cur);
            }

            // Undo downmix of decimator history
            if (band == XLL_BAND_1 && coeff_pre)
                for (int k = 1; k < XLL_DECI_HISTORY; k++)
                    dmix->deci_history[i][k] -=
                        mul15(c->deci_history[j][k], coeff_pre);
        }
    }
}

static void scale_down_mix(struct xll_chset *c, struct downmix *dmix, int band)
{
    struct xll_decoder *xll = c->decoder;
    struct xll_band *b = &c->bands[band];
    int nsamples = xll->nframesamples;
    int nsamples_log2 = xll->nframesamples_log2;

    if (!b->dmix_embedded)
        return;

    for (int i = 0; i < c->hier_m; i++) {
        int scale_cur = c->dmix_scale_cur[i];
        int scale_pre = c->dmix_scale_pre[i];
        int delta = scale_cur - scale_pre;

        // Scale down channel samples
        int *buf = dmix->samples[band][i];

        if (delta) {
            // Scaling coefficient interpolation
            int ramp = 1 << (nsamples_log2 - 1);
            for (int k = 0; k < nsamples; k++, ramp += delta)
                buf[k] = mul15(buf[k], scale_pre + (ramp >> nsamples_log2));
        } else if (scale_cur != (1 << 15)) {
            for (int k = 0; k < nsamples; k++)
                buf[k] = mul15(buf[k], scale_cur);
        }

        // Scale down decimator history
        if (band == XLL_BAND_1 && scale_pre != (1 << 15))
            for (int k = 1; k < XLL_DECI_HISTORY; k++)
                dmix->deci_history[i][k] =
                    mul15(dmix->deci_history[i][k], scale_pre);
    }
}

static int hier_down_mix(struct xll_decoder *xll)
{
    struct downmix dmix;
    struct xll_chset *c;
    int i, nchannels = 0;

    // Build channel vectors for active channel sets that are part of hierarchy
    for (i = 0, c = xll->chset; i < xll->nactivechsets; i++, c++) {
        if (!c->hier_chset)
            continue;

        if (nchannels + c->nchannels > SPEAKER_COUNT)
            return -DCADEC_EINVAL;

        for (int ch = 0; ch < c->nchannels; ch++) {
            dmix.samples[XLL_BAND_0][nchannels] =
                c->bands[XLL_BAND_0].msb_sample_buffer[ch];
            dmix.samples[XLL_BAND_1][nchannels] =
                c->bands[XLL_BAND_1].msb_sample_buffer[ch];
            dmix.deci_history[nchannels] = c->deci_history[ch];
            nchannels++;
        }
    }

    // Walk through hierarchial downmix embedded channel sets
    for (i = 0, c = xll->chset; i < xll->nchsets; i++, c++) {
        if (!is_hier_dmix_chset(c))
            continue;

        // Stop once enough channels are decoded for downmixed output
        if (c->hier_m > nchannels)
            c->hier_m = nchannels;
        if (c->hier_m == nchannels) {
            // Scale down preceding channels in all frequency bands
            scale_down_mix(c, &dmix, XLL_BAND_0);
            if (c->nfreqbands > 1)
                scale_down_mix(c, &dmix, XLL_BAND_1);
            break;
        }

        // Undo downmix of preceding channels in all frequency bands
        undo_down_mix(c, &dmix, XLL_BAND_0);
        if (c->nfreqbands > 1)
            undo_down_mix(c, &dmix, XLL_BAND_1);
    }

    return 0;
}

static int validate_hd_ma_frame(struct dcadec_context *dca)
{
    struct xll_decoder *xll = dca->xll;
    struct xll_chset *p = &xll->chset[0], *c;
    int i;

    // Validate the first (primary) channel set
    if (!p->primary_chset) {
        xll_err_once("The first channel set must be primary");
        return -DCADEC_ENOSUP;
    }

    if (!p->ch_mask_enabled && p->nchannels != 2) {
        xll_err_once("Unsupported number of channels with channel mask "
                     "disabled for primary channel set (%d)", p->nchannels);
        return -DCADEC_ENOSUP;
    }

    if (p->storage_bit_res != 16 && p->storage_bit_res != 24) {
        xll_err_once("Unsupported storage bit resolution for "
                     "primary channel set (%d)", p->storage_bit_res);
        return -DCADEC_ENOSUP;
    }

    if (p->pcm_bit_res > p->storage_bit_res) {
        xll_err("Invalid PCM bit resolution for primary channel set (%d > %d)",
                p->pcm_bit_res, p->storage_bit_res);
        return -DCADEC_EINVAL;
    }

    // Validate channel sets
    dca->has_residual_encoded = false;
    for (i = 0, c = xll->chset; i < xll->nactivechsets; i++, c++) {
        if (i > 0) {
            if (c->primary_chset) {
                xll_err_once("Multiple primary channel sets are not supported");
                return -DCADEC_ENOSUP;
            }

            if (!c->ch_mask_enabled) {
                xll_err_once("Secondary channel sets with channel mask "
                             "disabled are not supported");
                return -DCADEC_ENOSUP;
            }

            if (c->dmix_embedded && !c->hier_chset) {
                xll_err_once("Channel sets with embedded parallel downmix "
                             "are not supported");
                return -DCADEC_ENOSUP;
            }

            if (c->freq != p->freq || c->pcm_bit_res != p->pcm_bit_res
                || c->storage_bit_res != p->storage_bit_res
                || c->nfreqbands != p->nfreqbands) {
                xll_err_once("Channel sets with different audio "
                             "characteristics are not supported");
                return -DCADEC_ENOSUP;
            }
        }

        if (c->interpolate) {
            xll_err_once("Channel sets with sampling frequency modifier "
                         "are not supported");
            return -DCADEC_ENOSUP;
        }

        dca->has_residual_encoded |= c->residual_encode != (1 << c->nchannels) - 1;
    }

    // Verify that core is compatible if there are residual encoded channel sets
    if (dca->has_residual_encoded) {
        if (!(dca->packet & PACKET_CORE)) {
            xll_err("Residual encoded channels are present without core");
            return -DCADEC_EINVAL;
        }

        int rate = dca->core->sample_rate;
        int nsamples = dca->core->npcmblocks * NUM_PCMBLOCK_SAMPLES;

        // Double sampling frequency if needed
        if (p->freq == 96000 && rate == 48000) {
            rate *= 2;
            nsamples *= 2;
        }

        if (p->freq != rate) {
            xll_err_once("Sample rate mismatch between core (%d) and XLL (%d)",
                         rate, p->freq);
            return -DCADEC_ENOSUP;
        }

        if (xll->nframesamples != nsamples) {
            xll_err("Number of samples per frame mismatch between core (%d) "
                    "and XLL (%d)", nsamples, xll->nframesamples);
            return -DCADEC_EINVAL;
        }
    }

    return 0;
}

static void force_lossy_output(struct core_decoder *core, struct xll_chset *c)
{
    // Clear all band data
    xll_clear_band_data(c, XLL_BAND_0);
    if (c->nfreqbands > 1)
        xll_clear_band_data(c, XLL_BAND_1);

    // Replace non-residual encoded channels with lossy counterparts
    for (int ch = 0; ch < c->nchannels; ch++) {
        if (!(c->residual_encode & (1 << ch)))
            continue;
        int spkr = xll_map_ch_to_spkr(c, ch);
        if (spkr < 0)
            continue;
        int core_spkr = map_spkr_to_core_spkr(core, spkr);
        if (core_spkr < 0)
            continue;
        c->residual_encode &= ~(1 << ch);
    }
}

static int filter_residual_core_frame(struct dcadec_context *dca)
{
    struct core_decoder *core = dca->core;
    struct xll_decoder *xll = dca->xll;
    int i, ret, flags = DCADEC_FLAG_CORE_BIT_EXACT | DCADEC_FLAG_KEEP_DMIX_6CH;
    struct xll_chset *c;

    // Double sampling frequency if needed
    if (xll->chset->freq == 96000 && core->sample_rate == 48000)
        flags |= DCADEC_FLAG_CORE_SYNTH_X96;

    // Filter core frame
    if ((ret = core_filter(core, flags)) < 0) {
        dca->core_residual_valid = false;
        return ret;
    }

    // Force lossy downmixed output if this is the first core frame since
    // the last time history was cleared, or XLL decoder is recovering from sync loss
    if ((dca->has_residual_encoded && !dca->core_residual_valid && xll->nchsets > 1) ||
        (dca->packet & PACKET_RECOVERY)) {
        for (i = 0, c = xll->chset; i < xll->nchsets; i++, c++) {
            if (i < xll->nactivechsets)
                force_lossy_output(core, c);

            if (!c->primary_chset)
                c->dmix_embedded = false;
        }

        xll->scalable_lsbs = false;
        xll->fixed_lsb_width = 0;
    }

    dca->core_residual_valid = true;
    return 0;
}

static int combine_residual_core_frame(struct dcadec_context *dca,
                                       struct xll_chset *c)
{
    struct core_decoder *core = dca->core;
    struct xll_decoder *xll = dca->xll;
    int nsamples = xll->nframesamples;
    int nsamples_log2 = xll->nframesamples_log2;

    if (c->freq != core->output_rate)
        return -DCADEC_EINVAL;

    if (nsamples != core->npcmsamples)
        return -DCADEC_EINVAL;

    // See if this channel set is downmixed and find the next channel set in
    // hierarchy. If downmixed, undo core pre-scaling before combining with
    // residual (residual is not scaled).
    struct xll_chset *o = find_next_hier_dmix_chset(c);

    // Reduce core bit width and combine with residual
    for (int ch = 0; ch < c->nchannels; ch++) {
        if (c->residual_encode & (1 << ch))
            continue;

        int spkr = xll_map_ch_to_spkr(c, ch);
        if (spkr < 0)
            return -DCADEC_EINVAL;

        int core_spkr = map_spkr_to_core_spkr(core, spkr);
        if (core_spkr < 0)
            return -DCADEC_EINVAL;

        int shift = 24 - c->pcm_bit_res;
        // Account for LSB width
        if (xll->scalable_lsbs)
            shift += xll_get_lsb_width(c, XLL_BAND_0, ch);
        int round = shift > 0 ? 1 << (shift - 1) : 0;

        int *dst = c->bands[XLL_BAND_0].msb_sample_buffer[ch];
        int *src = core->output_samples[core_spkr];
        if (o) {
            // Undo embedded core downmix pre-scaling
            int scale_inv_cur = o->dmix_scale_inv_cur[c->hier_m + ch];
            int scale_inv_pre = o->dmix_scale_inv_pre[c->hier_m + ch];
            int delta = scale_inv_cur - scale_inv_pre;

            if (delta) {
                // Scaling coefficient interpolation
                int ramp = 1 << (nsamples_log2 - 1);
                for (int n = 0; n < nsamples; n++, ramp += delta) {
                    int scale_inv = scale_inv_pre + (ramp >> nsamples_log2);
                    dst[n] += clip23((mul16(src[n], scale_inv) + round) >> shift);
                }
            } else {
                for (int n = 0; n < nsamples; n++)
                    dst[n] += clip23((mul16(src[n], scale_inv_cur) + round) >> shift);
            }
        } else {
            // No downmix scaling
            for (int n = 0; n < nsamples; n++)
                dst[n] += (src[n] + round) >> shift;
        }
    }

    return 0;
}

static int filter_hd_ma_frame(struct dcadec_context *dca)
{
    struct xll_decoder *xll = dca->xll;
    struct xll_chset *p = &xll->chset[0], *c;
    int ret, i;

    // Filter core frame if present
    if (dca->packet & PACKET_CORE)
        if ((ret = filter_residual_core_frame(dca)) < 0)
            return ret;

    // Prepare downmixing coefficients for all channel sets
    for (i = xll->nchsets - 1, c = &xll->chset[i]; i >= 0; i--, c--) {
        // Pre-scale by next channel set in hierarchy
        if (is_hier_dmix_chset(c)) {
            struct xll_chset *o = find_next_hier_dmix_chset(c);
            if (o)
                prescale_down_mix(c, o);
        }

        // Flip buffers and mark downmix coefficients valid for the next frame
        c->dmix_coeffs_signature = XLL_DMIX_SIGNATURE(c);
        c->dmix_coeffs_parity ^= true;
    }

    // Process frequency bands for active channel sets
    for (i = 0, c = xll->chset; i < xll->nactivechsets; i++, c++) {
        xll_filter_band_data(c, XLL_BAND_0);

        // Check for residual encoded channel set
        if (c->residual_encode != (1 << c->nchannels) - 1)
            if ((ret = combine_residual_core_frame(dca, c)) < 0)
                return ret;

        // Assemble MSB and LSB parts after combining with core
        if (xll->scalable_lsbs)
            xll_assemble_msbs_lsbs(c, XLL_BAND_0);

        if (c->nfreqbands > 1) {
            xll_filter_band_data(c, XLL_BAND_1);
            xll_assemble_msbs_lsbs(c, XLL_BAND_1);
        }
    }

    // Undo hierarchial downmix and/or apply scaling
    if (xll->nchsets > 1 && (ret = hier_down_mix(xll)) < 0)
        return ret;

    // Assemble frequency bands 0 and 1 for active channel sets
    if (xll->nfreqbands > 1 && (ret = xll_assemble_freq_bands(xll)) < 0)
        return ret;

    // Output speaker map and channel mask
    int *spkr_map[SPEAKER_COUNT] = { NULL };
    int ch_mask = 0;

    // Fake up channel mask for primary channel set if needed for LtRt decoding
    if (!p->ch_mask_enabled) {
        if (p->nchannels == 2)
            p->ch_mask = SPEAKER_LAYOUT_STEREO;
        else
            return -DCADEC_EINVAL;
    }

    // Build the output speaker map
    for (i = 0, c = xll->chset; i < xll->nactivechsets; i++, c++) {
        for (int ch = 0; ch < c->nchannels; ch++) {
            int spkr = xll_map_ch_to_spkr(c, ch);
            if (spkr < 0)
                return -DCADEC_EINVAL;
            if (spkr_map[spkr])
                return -DCADEC_EINVAL;
            spkr_map[spkr] = c->out_sample_buffer[ch];
        }
        ch_mask |= c->ch_mask;
    }

    // Normalize to regular 5.1 layout if downmixing
    if (dca->flags & DCADEC_FLAG_KEEP_DMIX_MASK) {
        if (ch_mask & SPEAKER_MASK_Lss) {
            spkr_map[SPEAKER_Ls] = spkr_map[SPEAKER_Lss];
            ch_mask = (ch_mask & ~SPEAKER_MASK_Lss) | SPEAKER_MASK_Ls;
        }
        if (ch_mask & SPEAKER_MASK_Rss) {
            spkr_map[SPEAKER_Rs] = spkr_map[SPEAKER_Rss];
            ch_mask = (ch_mask & ~SPEAKER_MASK_Rss) | SPEAKER_MASK_Rs;
        }
    }

    // Downmix primary channel set to Lo/Ro
    if (dca->flags & DCADEC_FLAG_KEEP_DMIX_2CH) {
        if ((ret = down_mix_prim_chset(dca,
                                       p->dmix_embedded,
                                       p->dmix_type,
                                       p->dmix_coeff_cur,
                                       p->dmix_coeff_pre,
                                       spkr_map,
                                       xll->nframesamples << (xll->nfreqbands - 1),
                                       &ch_mask)) < 0)
            return ret;
    }

    // Reorder sample buffer pointers
    int nchannels;
    if ((nchannels = reorder_samples(dca, spkr_map, ch_mask)) <= 0)
        return -DCADEC_EINVAL;

    dca->nframesamples = xll->nframesamples << (xll->nfreqbands - 1);
    dca->sample_rate = p->freq << (xll->nfreqbands - 1);
    dca->bits_per_sample = p->storage_bit_res;
    dca->profile = DCADEC_PROFILE_HD_MA;

    // Shift and clip samples to account for storage bit width
    bool clipped = shift_and_clip(dca, nchannels, p->storage_bit_res, p->pcm_bit_res);

    // Warn if this frame has not been decoded losslessly
    if ((dca->packet & PACKET_RECOVERY) || xll->nfailedsegs > 0)
        return DCADEC_WXLLLOSSY;

    // Warn if clipping was detected in lossless output
    if (clipped && !(dca->flags & DCADEC_FLAG_KEEP_DMIX_MASK))
        return DCADEC_WXLLCLIPPED;

    return 0;
}

static int alloc_core_decoder(struct dcadec_context *dca)
{
    if (!dca->core) {
        if (!(dca->core = ta_znew(dca, struct core_decoder)))
            return -DCADEC_ENOMEM;
        dca->core->ctx = dca;
        dca->core->x96_rand = 1;
    }
    return 0;
}

static int alloc_exss_parser(struct dcadec_context *dca)
{
    if (!dca->exss) {
        if (!(dca->exss = ta_znew(dca, struct exss_parser)))
            return -DCADEC_ENOMEM;
        dca->exss->ctx = dca;
    }
    return 0;
}

static int alloc_xll_decoder(struct dcadec_context *dca)
{
    if (!dca->xll) {
        if (!(dca->xll = ta_znew(dca, struct xll_decoder)))
            return -DCADEC_ENOMEM;
        dca->xll->ctx = dca;
        dca->xll->flags = dca->flags;
    }
    return 0;
}

DCADEC_API int dcadec_context_parse(struct dcadec_context *dca, uint8_t *data, size_t size)
{
    int status = 0, ret;

    if (!dca || !data || size < 4 || size > MAX_PACKET_SIZE || ((uintptr_t)data & 3))
        return -DCADEC_EINVAL;

    int prev_packet = dca->packet;
    dca->packet = 0;

    // Parse backward compatible core sub-stream
    if (DCA_MEM32NE(data) == DCA_32BE_C(SYNC_WORD_CORE)) {
        if ((ret = alloc_core_decoder(dca)) < 0)
            return ret;
        if ((ret = core_parse(dca->core, data, size, dca->flags, NULL)) < 0) {
            dca->core_residual_valid = false;
            return ret;
        }
        if (ret > status)
            status = ret;

        dca->packet |= PACKET_CORE;

        // EXXS data must be aligned on 4-byte boundary by the caller
        size_t frame_size = DCA_ALIGN(dca->core->frame_size, 4);
        if (size - 4 > frame_size) {
            data += frame_size;
            size -= frame_size;
        }
    }

    struct exss_asset *asset = NULL;

    // Parse extension sub-stream (EXSS)
    if (DCA_MEM32NE(data) == DCA_32BE_C(SYNC_WORD_EXSS)) {
        if ((ret = alloc_exss_parser(dca)) < 0)
            return ret;
        if ((ret = exss_parse(dca->exss, data, size)) < 0) {
            if (dca->flags & DCADEC_FLAG_STRICT)
                return ret;
            status = DCADEC_WEXSSFAILED;
        } else {
            dca->packet |= PACKET_EXSS;
            asset = &dca->exss->assets[0];
        }
    }

    // Parse coding components in the first EXSS asset
    if (dca->packet & PACKET_EXSS) {
        // Parse core component in EXSS
        if (!(dca->packet & PACKET_CORE) && (asset->extension_mask & EXSS_CORE)) {
            if ((ret = alloc_core_decoder(dca)) < 0)
                return ret;
            if ((ret = core_parse(dca->core, data, size, dca->flags, asset)) < 0) {
                dca->core_residual_valid = false;
                return ret;
            }
            if (ret > status)
                status = ret;

            dca->packet |= PACKET_CORE;
        }

        // Parse XLL component in EXSS
        if (!(dca->flags & DCADEC_FLAG_CORE_ONLY) && (asset->extension_mask & EXSS_XLL)) {
            if ((ret = alloc_xll_decoder(dca)) < 0)
                return ret;
            if ((ret = xll_parse(dca->xll, data, asset)) < 0) {
                // Conceal XLL synchronization error
                if (ret == -DCADEC_ENOSYNC &&
                    (prev_packet & PACKET_XLL) &&
                    (dca->packet & PACKET_CORE)) {
                    dca->packet |= PACKET_XLL | PACKET_RECOVERY;
                    status = DCADEC_WXLLSYNCERR;
                } else {
                    if (dca->flags & DCADEC_FLAG_STRICT)
                        return ret;
                    status = DCADEC_WXLLFAILED;
                }
            } else {
                dca->packet |= PACKET_XLL;
                if (dca->xll->nfailedsegs)
                    status = DCADEC_WXLLBANDERR;
            }
        }
    }

    if (!dca->packet)
        return -DCADEC_ENOSYNC;

    // Parse core extensions in EXSS or backward compatible core sub-stream
    if (!(dca->flags & DCADEC_FLAG_CORE_ONLY) && (dca->packet & PACKET_CORE)) {
        if ((ret = core_parse_exss(dca->core, data, dca->flags, asset)) < 0)
            return ret;
        if (ret > status)
            status = ret;
    }

    return status;
}

DCADEC_API struct dcadec_core_info *dcadec_context_get_core_info(struct dcadec_context *dca)
{
    if (dca && (dca->packet & PACKET_CORE))
        return core_get_info(dca->core);
    return NULL;
}

DCADEC_API void dcadec_context_free_core_info(struct dcadec_core_info *info)
{
    ta_free(info);
}

DCADEC_API struct dcadec_exss_info *dcadec_context_get_exss_info(struct dcadec_context *dca)
{
    if (dca) {
        if (dca->packet & PACKET_EXSS)
            return exss_get_info(dca->exss);
        if (dca->packet & PACKET_CORE)
            return core_get_info_exss(dca->core);
    }
    return NULL;
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

    if (!(dca->packet & PACKET_FILTERED)) {
        if (dca->packet & PACKET_XLL) {
            if ((ret = validate_hd_ma_frame(dca)) < 0) {
                if (dca->flags & DCADEC_FLAG_STRICT)
                    return ret;
                if (!(dca->packet & PACKET_CORE))
                    return ret;
                if ((ret = filter_core_frame(dca)) < 0)
                    return ret;
                ret = DCADEC_WXLLCONFERR;
            } else {
                if ((ret = filter_hd_ma_frame(dca)) < 0)
                    return ret;
            }
        } else if (dca->packet & PACKET_CORE) {
            if ((ret = filter_core_frame(dca)) < 0)
                return ret;
        } else {
            return -DCADEC_EINVAL;
        }
        dca->status = ret;
        dca->packet |= PACKET_FILTERED;
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
    return dca->status;
}

DCADEC_API void dcadec_context_clear(struct dcadec_context *dca)
{
    if (dca) {
        core_clear(dca->core);
        xll_clear(dca->xll);
        dca->core_residual_valid = false;
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

DCADEC_API void dcadec_context_set_log_cb(struct dcadec_context *dca,
                                          dcadec_log_cb log_cb,
                                          void *log_cbarg)
{
    if (dca) {
        dca->log_cb = log_cb;
        dca->log_cbarg = log_cbarg;
        dca->log_shown = 0;
    }
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

    static const char * const warnings[] = {
        "Failed to parse core auxiliary data",
        "Failed to parse core extension",
        "Failed to parse EXSS",
        "Failed to parse XLL",
        "XLL synchronization error",
        "XLL frequency band error",
        "XLL configuration error",
        "Clipping detected in XLL output",
        "XLL output not lossless"
    };

    if (errnum < 0) {
        unsigned int err = -errnum - 1;
        if (err < dca_countof(errors))
            return errors[err];
        else
            return "Unspecified error";
    } else if (errnum > 0) {
        unsigned int warn = errnum - 1;
        if (warn < dca_countof(warnings))
            return warnings[warn];
        else
            return "Unspecified warning";
    } else {
        return "No error";
    }
}

DCADEC_API unsigned int dcadec_version(void)
{
    return DCADEC_API_VERSION;
}

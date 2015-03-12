/*
 * This file is part of dcadec.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef DCA_CONTEXT_H
#define DCA_CONTEXT_H

/**@{*/
#define DCADEC_EINVAL       1   /**< Invalid argument */
#define DCADEC_EBADDATA     2   /**< Invalid bitstream format */
#define DCADEC_EBADCRC      3   /**< CRC check failed */
#define DCADEC_EBADREAD     4   /**< Bitstream navigation error */
#define DCADEC_ENOSYNC      5   /**< Synchronization error */
#define DCADEC_ENOSUP       6   /**< Unsupported feature */
#define DCADEC_ENOMEM       7   /**< Memory allocation error */
#define DCADEC_EOVERFLOW    8   /**< PCM output overflow */
#define DCADEC_EIO          9   /**< I/O error */
#define DCADEC_EFAIL       32   /**< Unspecified error */
/**@}*/

/**@{*/
/** Decode DTS core only without extensions */
#define DCADEC_FLAG_CORE_ONLY           0x01

/** Force bit exact DTS core decoding */
#define DCADEC_FLAG_CORE_BIT_EXACT      0x02

/** Force DTS core synthesis using X96 filter */
#define DCADEC_FLAG_CORE_SYNTH_X96      0x04

/** Force DTS core bit width reducion to source PCM resolution */
#define DCADEC_FLAG_CORE_SOURCE_PCM_RES 0x08

/* Use FIR filter for floating point DTS core LFE channel interpolation */
#define DCADEC_FLAG_CORE_LFE_FIR        0x10

/** Extract embedded 2.0 downmix (NOT YET IMPLEMENTED) */
#define DCADEC_FLAG_KEEP_DMIX_2CH       0x20

/** Extract embedded 5.1 downmix (NOT YET IMPLEMENTED) */
#define DCADEC_FLAG_KEEP_DMIX_6CH       0x40

/** Output native DTS channel layout, not WAVEFORMATEX layout */
#define DCADEC_FLAG_NATIVE_LAYOUT       0x80
/**@}*/

/**@{*/
#define DCADEC_PROFILE_UNKNOWN  0       /**< Unknown Profile */
#define DCADEC_PROFILE_DS       0x01    /**< Digital Surround */
#define DCADEC_PROFILE_DS_96_24 0x02    /**< Digital Surround 96/24 */
#define DCADEC_PROFILE_DS_ES    0x04    /**< Digital Surround ES */
#define DCADEC_PROFILE_HD_HRA   0x08    /**< High-Resolution Audio */
#define DCADEC_PROFILE_HD_MA    0x10    /**< Master Audio */
#define DCADEC_PROFILE_EXPRESS  0x20    /**< Express */
/**@}*/

/**
 * Size in bytes of empty padding that must be present after the end of input
 * buffer. libdcadec may overread the input buffer up to this number of bytes.
 */
#define DCADEC_BUFFER_PADDING   8

struct dcadec_context;

/**
 * Parse DTS packet. Caller must have already established byte stream
 * synchronization. Packet must start with a valid 32-bit sync word.
 * EXSS frame must be aligned on 4-byte boundary if present in the packet.
 *
 * @param dca   Pointer to decoder context.
 *
 * @param data  Pointer to packet data buffer. Buffer must be aligned on 4-byte
 *              boundary and padded at the end with DCADEC_BUFFER_PADDING bytes.
 *
 * @param size  Size in bytes of packet data. Size should not include padding.
 *
 * @return      0 on success, negative error code on failure.
 */
int dcadec_context_parse(struct dcadec_context *dca, uint8_t *data, size_t size);

/**
 * Get information about DTS core payload of the parsed packet. All parameters
 * except decoder context are optional and can be NULL.
 *
 * @param dca           Pointer to decoder context.
 *
 * @param nchannels     Filled with number of primary audio channels.
 *
 * @param lfe_present   Filled with LFE channel presence flag. 0 - no LFE
 *                      channel present, 1 - LFE channel with 128 samples
 *                      decimation, 2 - LFE channel with 64 samples decimation.
 *
 * @param sample_rate       Filled with audio sample rate in Hz.
 *
 * @param source_pcm_res    Filled with source PCM resolution in bits.
 *
 * @param es_format         Filled with boolean value indicating whether
 *                          encoded stream is mastered in ES format or not.
 *
 * @param bit_rate          Filled with encoded stream bit rate in bytes per
 *                          second. If the bit rate is unknown, negative
 *                          value is returned.
 *
 * @return                  0 on success, negative error code on failure.
 */
int dcadec_context_core_info(struct dcadec_context *dca, int *nchannels,
                             int *lfe_present, int *sample_rate,
                             int *source_pcm_res, int *es_format, int *bit_rate);

/**
 * Get information about extension sub-stream payload of the parsed packet.
 * All parameters except decoder context are optional and can be NULL.
 *
 * @param dca               Pointer to decoder context.
 *
 * @param nchannels         Filled with number of audio channels encoded in all
 *                          sub-streams.
 *
 * @param sample_rate       Filled with maximum audio sample rate in Hz.
 *
 * @param bits_per_sample   Filled with highest PCM resolution in bits.
 *
 * @param profile           Filled with type of DTS profile encoded.
 *
 * @return                  0 on success, negative error code on failure.
 */
int dcadec_context_exss_info(struct dcadec_context *dca, int *nchannels,
                             int *sample_rate, int *bits_per_sample,
                             int *profile);

/**
 * Filter the parsed packet and return per-channel PCM data. All parameters
 * except decoder context are optional and can be NULL. This function should
 * be called exactly once after successfull call to dcadec_context_parse().
 *
 * @param dca       Pointer to decoder context.
 *
 * @param samples   Filled with address of array of pointers to planes
 *                  containing PCM data for active channels. This data is only
 *                  valid until the next call to any libdcadec function.
 *                  Returned array is tightly packed, there are no gaps for
 *                  missing channels. Use channel_mask to determine total number
 *                  of channels and size of returned array. By default channels
 *                  are ordered according to WAVEFORMATEX specification, but if
 *                  DCADEC_FLAG_NATIVE_LAYOUT flag was set when creating decoder
 *                  context, returned channels are in native DTS order.
 *
 * @param nsamples  Filled with number of PCM samples in each returned plane.
 *
 * @param channel_mask  Filled with bit mask indicating active channels. 1 at
 *                      the given bit position (counting from the least
 *                      significant bit) means that the channel is present in
 *                      the array of pointers to planes, 0 otherwise. Number of
 *                      bits set to 1 indicates the total number of planes
 *                      returned.
 *
 * @param sample_rate       Filled with decoded audio sample rate in Hz.
 *
 * @param bits_per_sample   Filled with decoded audio PCM resolution in bits.
 *
 * @param profile           Filled with type of DTS profile actually decoded.
 *                          This can be different from encoded profile since
 *                          certain extensions may be not decoded.
 *
 * @return                  0 on success, negative error code on failure.
 */
int dcadec_context_filter(struct dcadec_context *dca, int ***samples,
                          int *nsamples, int *channel_mask, int *sample_rate,
                          int *bits_per_sample, int *profile);

/**
 * Clear all inter-frame history of the decoder.
 *
 * @param dca   Pointer to decoder context.
 */
void dcadec_context_clear(struct dcadec_context *dca);

/**
 * Create DTS decoder context.
 *
 * @param flags Any number of DCADEC_FLAG_* constants OR'ed together.
 *
 * @return      Pointer to decoder context on success, NULL on failure.
 */
struct dcadec_context *dcadec_context_create(int flags);

/**
 * Destroy DTS decoder context.
 *
 * @param dca   Pointer to decoder context.
 */
void dcadec_context_destroy(struct dcadec_context *dca);

#endif

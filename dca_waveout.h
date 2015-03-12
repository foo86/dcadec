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

#ifndef WAVEOUT_H
#define WAVEOUT_H

struct dcadec_waveout;

int dcadec_waveout_write(struct dcadec_waveout *wave, int **samples,
                         int nsamples, int channel_mask, int sample_rate,
                         int bits_per_sample);
struct dcadec_waveout *dcadec_waveout_open(const char *name);
void dcadec_waveout_close(struct dcadec_waveout *wave);

#endif

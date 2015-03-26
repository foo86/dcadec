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
#include "interpolator.h"

struct interpolator_data *interpolator_init(struct core_decoder *parent)
{
    struct interpolator_data *data = ta_new(parent, struct interpolator_data);
    if (!data)
        return NULL;

    for (int i = 0, k = 0; i < 32; i++)
        for (int j = 0; j < 32; j++)
            data->cos_mod_32[k++] = 0.250 * cos((2 * i + 1) * (2 * j + 1) * M_PI / 128);

    for (int i = 0, k = 0; i < 64; i++)
        for (int j = 0; j < 64; j++)
            data->cos_mod_64[k++] = 0.125 * cos((2 * i + 1) * (2 * j + 1) * M_PI / 256);

    return data;
}

struct interpolator *interpolator_create(struct interpolator_data *parent, int flags)
{
    struct interpolator *dsp = ta_new(parent, struct interpolator);
    if (!dsp)
        return NULL;

    dsp->data = parent;
    dsp->history = ta_znew_array_size(dsp,
        (flags & DCADEC_FLAG_CORE_BIT_EXACT) ? sizeof(int) : sizeof(double),
        (flags & DCADEC_FLAG_CORE_SYNTH_X96) ? 1024 : 512);
    if (!dsp->history) {
        ta_free(dsp);
        return NULL;
    }

    if (flags & DCADEC_FLAG_CORE_BIT_EXACT) {
        if (flags & DCADEC_FLAG_CORE_SYNTH_X96)
            dsp->interpolate = interpolate_sub64_fixed;
        else
            dsp->interpolate = interpolate_sub32_fixed;
    } else {
        if (flags & DCADEC_FLAG_CORE_SYNTH_X96)
            dsp->interpolate = interpolate_sub64_float;
        else
            dsp->interpolate = interpolate_sub32_float;
    }

    return dsp;
}

void interpolator_clear(struct interpolator *dsp)
{
    if (dsp)
        memset(dsp->history, 0, ta_get_size(dsp->history));
}

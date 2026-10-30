/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <math.h>
#include <stdio.h>

#include "libavcodec/aac.h"
#include "libavcodec/aactab.h"
#include "libavcodec/avcodec.h"
#include "libavcodec/psymodel.h"

static int check_mask(int mode, int short_windows, int quiet)
{
    AVCodecContext avctx = {
        .codec_id    = AV_CODEC_ID_AAC,
        .sample_rate = 48000,
        .bit_rate    = 128000,
        .cutoff      = 20000,
        .ch_layout   = AV_CHANNEL_LAYOUT_MONO,
        .flags       = mode == 1 ? AV_CODEC_FLAG_QSCALE : 0,
    };
    FFPsyContext psy = { 0 };
    FFPsyWindowInfo wi = {
        .num_windows = short_windows ? 8 : 1,
        .window_type = { short_windows ? EIGHT_SHORT_SEQUENCE : ONLY_LONG_SEQUENCE },
    };
    const uint16_t *offsets[] = { ff_swb_offset_1024[3], ff_swb_offset_128[3] };
    const int num_bands[] = { ff_aac_num_swb_1024[3], ff_aac_num_swb_128[3] };
    uint8_t sizes[2][64], group_map[] = { 0 };
    const uint8_t *bands[] = { sizes[0], sizes[1] };
    float coeffs[1024];
    const float *channels[] = { coeffs };
    int checked = 0, finite = 0, masked = 0, limited = 0, ret = 0;

    for (int i = 0; i < 2; i++)
        for (int g = 0; g < num_bands[i]; g++)
            sizes[i][g] = offsets[i][g + 1] - offsets[i][g];
    for (int i = 0; i < 1024; i++)
        coeffs[i] = quiet ? 1e-8f : 1e4f;

    if (ff_psy_init(&psy, &avctx, 2, bands, num_bands, 1, group_map, 20000) < 0) {
        ret = 1;
        goto end;
    }
    psy.unbounded_pe = mode == 2;
    psy.model->analyze(&psy, 0, channels, &wi);

    for (int w = 0; w < wi.num_windows; w++) {
        for (int g = 0; g < num_bands[short_windows]; g++) {
            const FFPsyBand *band = &psy.ch[0].psy_bands[w * 16 + g];
            float freq = offsets[short_windows][g] * 24000.0f /
                         (short_windows ? 128 : 1024);
            float ceiling = band->energy * (mode ? 0.1f : 0.251189f);

            if (freq < 8000 || freq >= 20000 || band->energy <= 0)
                continue;
            checked++;
            finite  += !!isfinite(band->threshold);
            masked  += band->threshold >= band->energy;
            limited += band->threshold <= ceiling * 1.00001f;
        }
    }
    /* Compare stable counts in FATE rather than platform-dependent floats. */
    printf("%s %s %s: bands %d, finite %d, masked %d, limited %d\n",
           mode == 0 ? "CBR" : mode == 1 ? "VBR" : "ABR",
           short_windows ? "short" : "long", quiet ? "quiet" : "audible",
           checked, finite, masked, limited);
end:
    ff_psy_end(&psy);
    return ret;
}

int main(void)
{
    for (int mode = 0; mode < 3; mode++)
        for (int short_windows = 0; short_windows < 2; short_windows++)
            for (int quiet = 0; quiet < 2; quiet++)
                if (check_mask(mode, short_windows, quiet))
                    return 1;
    return 0;
}

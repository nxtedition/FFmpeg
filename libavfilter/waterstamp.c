/*
 * WATERSTAMP v1 -- the emitters' shared anchor and the detectors' frame decoder.
 * Copyright (c) 2026 nxtedition
 *
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
#include <string.h>

#include "libavutil/avstring.h"
#include "libavutil/common.h"
#include "libavutil/mem.h"
#include "libavutil/thread.h"
#include "libavutil/time.h"
#include "waterstamp.h"

#define MAX_GROUPS 32

static AVMutex anchor_lock = AV_MUTEX_INITIALIZER;
static struct {
    char    name[64];
    int64_t t0_us;
} anchors[MAX_GROUPS];
static int nb_anchors;

int64_t ff_waterstamp_wall_anchor(const char *group, int64_t pts_us)
{
    int64_t t0 = av_gettime() - pts_us;

    if (!group)
        group = "";
    ff_mutex_lock(&anchor_lock);
    for (int i = 0; i < nb_anchors; i++)
        if (!strcmp(anchors[i].name, group)) {
            t0 = anchors[i].t0_us;
            goto done;
        }
    if (nb_anchors < MAX_GROUPS) {
        av_strlcpy(anchors[nb_anchors].name, group, sizeof(anchors[nb_anchors].name));
        anchors[nb_anchors++].t0_us = t0;
    }
done:
    ff_mutex_unlock(&anchor_lock);
    return t0;
}

int ff_ws_framedec_init(WSFrameDec *d, int id, const uint8_t *key, size_t keylen)
{
    uint8_t bits[WS_BITS];

    memset(d, 0, sizeof(*d));
    d->cw  = av_malloc((size_t)WS_FD_HYP * WS_SLOTS);
    d->acc = av_calloc((size_t)WS_SLOTS * WS_FD_HYP, sizeof(*d->acc));
    if (!d->cw || !d->acc) {
        ff_ws_framedec_uninit(d);
        return AVERROR(ENOMEM);
    }
    for (int q = 0; q < WS_FD_HYP; q++) {
        ws_frame_bits(2 * q, id, bits, key, keylen);
        for (int i = 0; i < WS_SLOTS; i++)
            d->cw[q * WS_SLOTS + i] = ws_symbol(bits, i);
    }
    return 0;
}

void ff_ws_framedec_reset(WSFrameDec *d)
{
    uint8_t *cw = d->cw;
    float  *acc = d->acc;
    memset(d, 0, sizeof(*d));
    d->cw  = cw;
    d->acc = acc;
    if (acc)
        memset(acc, 0, (size_t)WS_SLOTS * WS_FD_HYP * sizeof(*acc));
}

void ff_ws_framedec_uninit(WSFrameDec *d)
{
    av_freep(&d->cw);
    av_freep(&d->acc);
}

int ff_ws_framedec_push(WSFrameDec *d, const float *z, double pos, WSFrameResult *res)
{
    const int k = d->nslot % WS_SLOTS;
    int64_t f, j;
    int a, adv, best_q = 0, ga = 0, ev = WS_FD_NONE;
    float *acc, best = -1e30f, norm;

    /* A slot's weight is bounded: a near-silent slot with an underestimated
     * noise floor would otherwise outvote every other slot. At high SNR
     * the sent symbol is still far above the rest; at low SNR the clip
     * rarely acts. */
    if (z)
        for (int i = 0; i < WS_CSK_M; i++)
            d->z[k][i] = av_clipf(z[i], -WS_FD_ZMAX, WS_FD_ZMAX);
    else
        memset(d->z[k], 0, sizeof(d->z[k]));
    d->pos[k] = pos;
    d->nslot++;
    if (d->nslot < WS_SLOTS)
        return WS_FD_NONE;

    /* the frame that would have started 12 slots ago, in its alignment:
     * hypothesis q is the payload (2q) of frame 0 of that alignment, so
     * frame j carries payload 2 * ((q + 3j) mod 16384) */
    f   = d->nslot - WS_SLOTS;
    a   = f % WS_SLOTS;
    j   = f / WS_SLOTS;
    adv = (int)((3 * j) % WS_FD_HYP);
    acc = d->acc + (size_t)a * WS_FD_HYP;
    for (int q = 0; q < WS_FD_HYP; q++) {
        const uint8_t *c = d->cw + (size_t)((q + adv) & (WS_FD_HYP - 1)) * WS_SLOTS;
        float sc = 0.f;
        for (int i = 0; i < WS_SLOTS; i++)
            sc += d->z[(f + i) % WS_SLOTS][c[i]];
        acc[q] = WS_FD_LAMBDA * acc[q] + sc;
        if (acc[q] > best) {
            best   = acc[q];
            best_q = q;
        }
    }
    d->var[a] = WS_FD_LAMBDA * WS_FD_LAMBDA * d->var[a] + WS_SLOTS;
    d->nfr[a]++;
    norm = 1.f / sqrtf(d->var[a]);

    d->best[a]   = best * norm;
    d->best_q[a] = best_q;
    res->pos = d->pos[f % WS_SLOTS];

    /* A wrong alignment still half-matches a strong stamp and scores far
     * above noise, so decisions compare the best of all twelve. */
    for (int i = 1; i < WS_SLOTS; i++)
        if (d->best[i] > d->best[ga])
            ga = i;

    if (!d->lock) {
        if (d->nslot < 2 * WS_SLOTS || d->nfr[a] < 2 || ga != a || d->best[a] < WS_FD_LOCK)
            return WS_FD_NONE;
        d->lock   = 1;
        d->lock_a = a;
        d->lock_q = best_q;
        d->misses = 0;
        d->stat   = d->best[a];
        ev = WS_FD_LOCK_ON;
    } else if (a == d->lock_a) {
        d->stat = acc[d->lock_q] * norm;
        if (ga == a && best_q != d->lock_q && d->best[a] >= WS_FD_LOCK &&
            (d->stat < WS_FD_HOLD || d->best[a] > WS_FD_SWITCH * d->stat)) {
            d->lock_q = best_q;             /* the source time jumped */
            d->stat   = d->best[a];
            d->misses = 0;
            ev = WS_FD_JUMP;
        } else if (d->stat >= WS_FD_HOLD) {
            d->misses = 0;
            ev = WS_FD_FRAME;
        } else if (++d->misses >= WS_FD_MISSES) {
            d->lock = 0;
            return WS_FD_LOCK_OFF;
        } else {
            return WS_FD_NONE;
        }
    } else if (ga == a && d->best[a] >= WS_FD_LOCK &&
               (d->stat < WS_FD_HOLD || d->best[a] > WS_FD_SWITCH * d->stat)) {
        /* another alignment leads: a jump off the 6 s grid, or an early
         * lock on a half-matching alignment corrected */
        d->lock_a = a;
        d->lock_q = best_q;
        d->misses = 0;
        d->stat   = d->best[a];
        ev = WS_FD_JUMP;
    } else {
        return WS_FD_NONE;
    }
    res->payload = 2u * ((d->lock_q + adv) & (WS_FD_HYP - 1));
    res->stat    = d->stat;
    return ev;
}

int ff_ws_framedec_expected(const WSFrameDec *d, int back)
{
    int64_t n, f0;
    int idx;
    if (!d->lock || back >= d->nslot)
        return -1;
    n   = d->nslot - 1 - back;
    f0  = n - ((n - d->lock_a) % WS_SLOTS + WS_SLOTS) % WS_SLOTS;
    if (f0 < 0)
        return -1;
    idx = (int)((d->lock_q + 3 * (f0 / WS_SLOTS)) & (WS_FD_HYP - 1));
    return d->cw[(size_t)idx * WS_SLOTS + (n - f0)];
}

void ff_ws_framedec_unlock(WSFrameDec *d)
{
    d->lock = 0;
}

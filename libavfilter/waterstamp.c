/*
 * WATERSTAMP v1 -- state shared by the emitters of one process.
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

#include <string.h>

#include "libavutil/avstring.h"
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

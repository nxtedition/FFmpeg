/*
 * WATERSTAMP v1 -- inaudible spread-spectrum time stamp for programme audio.
 * The signal format was first specified as "NXTSTAMP v1" (nxtedition); the
 * filters and constants carry the vendor-neutral name.
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

/**
 * @file
 * Shared constants and bit-level helpers for the waterstamp emitter
 * (af_waterstamp.c) and the waterdetect detector (af_waterdetect.c).
 *
 * The waveform is a pure function of absolute source time: nothing is
 * carried between frames, so a restart resumes phase-continuous and two
 * emitters fed the same clock produce bit-identical output.
 *
 * Two layers share the band and are summed:
 *   Layer A  an unmodulated Gold code repeating every 500 ms (timing)
 *   Layer B  a second Gold code, cyclically shifted per 500 ms slot to
 *            carry 3 bits (8-ary cyclic shift keying); 12 slots = 36 bits
 *            = one frame every 6 s, aligned to t mod 6 == 0.
 * Both codes are selected by the source id, so several stamps coexist.
 *
 * Keyed mode: when a key is given, the two codes are pseudo-random +/-1
 * sequences derived from HMAC-SHA256(key, id, layer) instead of the public
 * Gold codes, and the 8-bit check field of every frame is a truncated
 * HMAC-SHA256 over the frame bits instead of CRC-8. A detector without the
 * key neither accepts nor sees the stamp, and a stamp made with the same id
 * under another key (or none) is rejected. Without a key everything is
 * public and interoperable.
 */

#ifndef AVFILTER_WATERSTAMP_H
#define AVFILTER_WATERSTAMP_H

#include <stdint.h>
#include <string.h>
#include "libavutil/error.h"
#include "libavutil/mem.h"
#include "libavutil/sha.h"

/* --- signal constants ------------------------------------------------ */
#define WS_VERSION     1
#define WS_SEQ_LEN     1023      /* chips per period, 2^10 - 1          */
#define WS_CHIP_RATE   2046      /* chip/s -> one period is exactly 500 ms */
#define WS_CARRIER     2000.0    /* Hz; occupied band 977..3023 Hz        */
#define WS_SLOT_US     500000    /* one period, one symbol                */
#define WS_SLOTS       12        /* slots per frame                       */
#define WS_FRAME_US    6000000   /* 6.000 s, aligned to t % 6 == 0        */
#define WS_FRAME_S     6
#define WS_BITS        36        /* 12 slots x 3 bits                     */
#define WS_CSK_M       8         /* 8-ary cyclic shift keying             */
#define WS_CSK_STEP    128       /* chips between adjacent shifts         */
#define WS_SYNC        0x34      /* 0b110100                              */
#define WS_SYNC_BITS   6
#define WS_ID_BITS     6
#define WS_PAYLOAD_BITS 15       /* seconds mod 32768, 9.1 h unambiguous  */
#define WS_PAYLOAD_MOD 32768
#define WS_NORM        2.0f      /* half-sine * carrier -> unit RMS       */

/* Detector analysis rates: 4 samples per chip in, 2 per chip at baseband. */
#define WS_ANALYSIS_RATE (4 * WS_CHIP_RATE)   /* 8184 Hz */
#define WS_BB_RATE       (2 * WS_CHIP_RATE)   /* 4092 Hz */
#define WS_BB_PERIOD     (2 * WS_SEQ_LEN)     /* 2046 baseband samples  */

/* Galois LFSR feedback masks over a 10-bit state. The two form a preferred
 * pair (the GPS C/A generator of IS-GPS-200): their cross-correlation takes
 * only the values -1, 63 and -65. Each source id owns a pair of Gold codes
 * built from them, so up to 64 sources coexist in one signal -- a mix of
 * stamped sources, or a feed stamped again per stage -- with every
 * cross-correlation bounded at 65/1023 (-23.9 dB). */
#define WS_LFSR_A      0x204u    /* x^10 + x^3 + 1                         */
#define WS_LFSR_G2     0x3A6u    /* x^10 + x^9 + x^8 + x^6 + x^3 + x^2 + 1 */
#define WS_MAX_IDS     64
#define WS_GOLD_A(id)  (2 * (id))  /* layer A code of source id             */
#define WS_GOLD_B(id)  (2 * (id) + 1)  /* layer B code of source id         */

/**
 * Fill seq[] with +/-1 from a Galois LFSR.
 *
 * @return 0 on success, AVERROR_BUG if the mask is not primitive. A
 *         non-primitive mask short-cycles and every downstream correlation
 *         silently degrades, so callers must check this at init.
 */
static inline int ws_mseq(uint16_t mask, int8_t *seq)
{
    uint16_t reg = 1;
    for (int i = 0; i < WS_SEQ_LEN; i++) {
        int lsb = reg & 1;
        seq[i] = lsb ? 1 : -1;
        reg >>= 1;
        if (lsb)
            reg ^= mask;
    }
    return reg == 1 ? 0 : AVERROR_BUG;   /* full period returns to seed */
}

/* SHA-256 over the concatenation of parts. */
static inline int ws_sha256(uint8_t out[32], const uint8_t *const parts[],
                            const size_t lens[], int n)
{
    struct AVSHA *sha = av_sha_alloc();
    if (!sha)
        return AVERROR(ENOMEM);
    av_sha_init(sha, 256);
    for (int i = 0; i < n; i++)
        av_sha_update(sha, parts[i], lens[i]);
    av_sha_final(sha, out);
    av_free(sha);
    return 0;
}

/* HMAC-SHA256(key, msg). */
static inline int ws_hmac(uint8_t out[32], const uint8_t *key, size_t keylen,
                          const uint8_t *msg, size_t msglen)
{
    uint8_t k[64] = { 0 }, ipad[64], opad[64], inner[32];
    const uint8_t *p[2];
    size_t l[2];
    int ret;

    if (keylen > 64) {
        p[0] = key; l[0] = keylen;
        if ((ret = ws_sha256(k, p, l, 1)) < 0)
            return ret;
    } else {
        memcpy(k, key, keylen);
    }
    for (int i = 0; i < 64; i++) {
        ipad[i] = k[i] ^ 0x36;
        opad[i] = k[i] ^ 0x5c;
    }
    p[0] = ipad; l[0] = 64; p[1] = msg;   l[1] = msglen;
    if ((ret = ws_sha256(inner, p, l, 2)) < 0)
        return ret;
    p[0] = opad; l[0] = 64; p[1] = inner; l[1] = 32;
    return ws_sha256(out, p, l, 2);
}

/* Keyed code: 1023 chips from HMAC-SHA256(key, "waterstamp-code" id layer block). */
static inline int ws_keyed_sequence(const uint8_t *key, size_t keylen, int id,
                                    int layer, int8_t *seq)
{
    uint8_t msg[18] = "waterstamp-code", digest[32];
    int n = 0, ret;
    msg[15] = id;
    msg[16] = layer;
    for (int block = 0; n < WS_SEQ_LEN; block++) {
        msg[17] = block;
        if ((ret = ws_hmac(digest, key, keylen, msg, sizeof(msg))) < 0)
            return ret;
        for (int i = 0; i < 256 && n < WS_SEQ_LEN; i++, n++)
            seq[n] = (digest[i >> 3] >> (7 - (i & 7))) & 1 ? 1 : -1;
    }
    return 0;
}

/* Gold code k of the family: a[i] * b[(i + k) mod N]. */
static inline void ws_gold(const int8_t *a, const int8_t *b, int k, int8_t *g)
{
    for (int i = 0; i < WS_SEQ_LEN; i++)
        g[i] = a[i] * b[(i + k) % WS_SEQ_LEN];
}

/**
 * Build the layer A and layer B sequences of source id: keyed sequences
 * when keylen > 0, else the public Gold code pair.
 * @return 0, AVERROR_BUG if a mask is not primitive, or an allocation error.
 */
static inline int ws_sequences(const uint8_t *key, size_t keylen, int id,
                               int8_t *seqA, int8_t *seqB)
{
    int8_t a[WS_SEQ_LEN], b[WS_SEQ_LEN];
    int ret;
    if (keylen) {
        if ((ret = ws_keyed_sequence(key, keylen, id, 0, seqA)) < 0)
            return ret;
        return ws_keyed_sequence(key, keylen, id, 1, seqB);
    }
    if ((ret = ws_mseq(WS_LFSR_A, a)) < 0 || (ret = ws_mseq(WS_LFSR_G2, b)) < 0)
        return ret;
    ws_gold(a, b, WS_GOLD_A(id), seqA);
    ws_gold(a, b, WS_GOLD_B(id), seqB);
    return 0;
}

/* CRC-8/ATM over a bit array: poly 0x07, init 0, no reflection. */
static inline uint8_t ws_crc8(const uint8_t *bits, int n)
{
    uint8_t crc = 0;
    for (int i = 0; i < n; i++) {
        uint8_t in = (bits[i] & 1) ^ (crc >> 7);
        crc = (uint8_t)(crc << 1);
        if (in)
            crc ^= 0x07;
    }
    return crc;
}

/* 8-bit check over the first n frame bits: CRC-8 without a key, the first
 * byte of HMAC-SHA256(key, "waterstamp-check" packed-bits) with one. */
static inline uint8_t ws_check8(const uint8_t *key, size_t keylen,
                                const uint8_t *bits, int n)
{
    uint8_t msg[16 + 8] = "waterstamp-check", digest[32];
    if (!keylen)
        return ws_crc8(bits, n);
    memset(msg + 16, 0, 8);
    for (int i = 0; i < n && i < 64; i++)
        msg[16 + (i >> 3)] |= (bits[i] & 1) << (7 - (i & 7));
    if (ws_hmac(digest, key, keylen, msg, sizeof(msg)) < 0)
        return ws_crc8(bits, n);          /* allocation failure: degrade, never crash */
    return digest[0];
}

/* Pack the 36 bits of the frame that starts at absolute second t0. */
static inline void ws_frame_bits(int64_t t0, unsigned id, uint8_t bits[WS_BITS],
                                 const uint8_t *key, size_t keylen)
{
    unsigned payload = (unsigned)(t0 & (WS_PAYLOAD_MOD - 1));
    int n = 0;
    for (int i = WS_SYNC_BITS - 1;    i >= 0; i--) bits[n++] = (WS_SYNC >> i) & 1;
    for (int i = WS_ID_BITS - 1;      i >= 0; i--) bits[n++] = (id       >> i) & 1;
    for (int i = WS_PAYLOAD_BITS - 1; i >= 0; i--) bits[n++] = (payload  >> i) & 1;
    bits[n++] = 0;                                 /* reserved */
    {
        uint8_t crc = ws_check8(key, keylen, bits, n);
        for (int i = 7; i >= 0; i--) bits[n++] = (crc >> i) & 1;
    }
}

/**
 * Unpack and validate a 36-bit frame.
 * @return 1 if sync word and CRC match (id and payload filled), else 0.
 */
static inline int ws_frame_parse(const uint8_t bits[WS_BITS], unsigned *id, unsigned *payload,
                                 const uint8_t *key, size_t keylen)
{
    unsigned v = 0;
    int n = 0;
    for (int i = 0; i < WS_SYNC_BITS; i++) v = (v << 1) | bits[n++];
    if (v != WS_SYNC)
        return 0;
    if (ws_check8(key, keylen, bits, WS_BITS - 8) != (uint8_t)((bits[28] << 7) | (bits[29] << 6) |
                                                   (bits[30] << 5) | (bits[31] << 4) |
                                                   (bits[32] << 3) | (bits[33] << 2) |
                                                   (bits[34] << 1) |  bits[35]))
        return 0;
    v = 0;
    for (int i = 0; i < WS_ID_BITS; i++) v = (v << 1) | bits[n++];
    *id = v;
    v = 0;
    for (int i = 0; i < WS_PAYLOAD_BITS; i++) v = (v << 1) | bits[n++];
    *payload = v;
    return 1;
}

/* Symbol for slot s: three bits, MSB first. */
static inline int ws_symbol(const uint8_t bits[WS_BITS], int s)
{
    return (bits[3*s] << 2) | (bits[3*s+1] << 1) | bits[3*s+2];
}

/**
 * Wall-clock anchor of an emitter group: the absolute time, in us, of media
 * timestamp 0. The first emitter of a group to ask sets it from the wall
 * clock and the frame it holds (at pts_us); every later one gets the same
 * value, so picture and sound stamped in one process share one timeline.
 */
int64_t ff_waterstamp_wall_anchor(const char *group, int64_t pts_us);

/* --- soft-decision frame decoder (shared by both detectors) -------------
 *
 * A frame is a codeword of 12 symbols fully determined by (id, key,
 * payload), and payloads of consecutive frames differ by exactly 6 s. So
 * instead of deciding each symbol and checking the CRC, the decoder scores
 * every codeword against the soft symbol metrics and accumulates the
 * scores along the payload sequence across frames: the metric is
 * maximum-likelihood over the whole code, and a weak stamp locks after
 * more frames instead of never. Payloads are even (frames start at
 * multiples of 6 s), so there are 16384 hypotheses per frame alignment.
 */
#define WS_FD_HYP     (WS_PAYLOAD_MOD / 2)
#define WS_FD_LAMBDA  0.75f     /* per-frame forgetting of the accumulator  */
#define WS_FD_LOCK    7.0f      /* normalised score to acquire a lock       */
#define WS_FD_HOLD    4.0f      /* ... and below which a held lock misses   */
#define WS_FD_MISSES  3         /* consecutive misses that drop the lock    */
#define WS_FD_ZMAX    3.0f      /* symbol metrics are clipped to +-this     */
#define WS_FD_SWITCH  1.3f      /* lead that moves a lock to another alignment */
#define WS_FD_MARGIN  0.3f      /* normalised lead over the runner-up       */
#define WS_FD_HIST    (2 * WS_SLOTS)
/* Both layers of a stamp have the same amplitude, so over the last two
 * frames the held codeword's metrics must add up to layer A's on the same
 * scale; cross-talk of another stamp lands in the two layers unrelated.
 * Measured at lock: genuine 0.85-1.10, cross-talk 0.3-0.5, up to 0.71 on
 * the frame the decoder picked it. */
#define WS_FD_RATIO   0.8f      /* to report a lock                         */
#define WS_FD_RHOLD   0.6f      /* to keep one                              */

enum { WS_FD_NONE, WS_FD_FRAME, WS_FD_LOCK_ON, WS_FD_JUMP, WS_FD_LOCK_OFF };

typedef struct WSFrameDec {
    uint8_t *cw;                        /* [WS_FD_HYP][WS_SLOTS] symbols       */
    float   *acc;                       /* [WS_SLOTS][WS_FD_HYP] scores         */
    float    var[WS_SLOTS];             /* accumulated noise variance           */
    int      nfr[WS_SLOTS];             /* frames accumulated per alignment     */
    float    z[WS_SLOTS][WS_CSK_M];     /* last 12 slots' metrics               */
    float    hz[WS_FD_HIST][WS_CSK_M];  /* last 24 slots, for the layer check   */
    float    hza[WS_FD_HIST];
    double   pos[WS_SLOTS];             /* caller's position of those slots     */
    int64_t  nslot;
    float    best[WS_SLOTS];            /* best normalised score per alignment  */
    int      best_q[WS_SLOTS];
    int      sure[WS_SLOTS];            /* best leads the runner-up clearly     */
    int      lock, lock_a, lock_q, misses;
    int      reported;                  /* the lock passed the layer check      */
    float    ratio;                     /* last layer check: symbols / layer A  */
    float    stat;                      /* normalised score of the last frame   */
} WSFrameDec;

typedef struct WSFrameResult {
    double   pos;                       /* position of the frame's first slot   */
    unsigned payload;
    float    stat;
} WSFrameResult;

int  ff_ws_framedec_init(WSFrameDec *d, int id, const uint8_t *key, size_t keylen);
void ff_ws_framedec_reset(WSFrameDec *d);
void ff_ws_framedec_uninit(WSFrameDec *d);
/**
 * Push one slot: z[] are the eight symbol metrics, unit variance under
 * noise and positive for the sent symbol; za is layer A's metric on the
 * same scale; z NULL marks a missed slot.
 * @return WS_FD_* event; res is filled for every event but WS_FD_NONE.
 *         Locks are only reported once they pass the layer check.
 */
int  ff_ws_framedec_push(WSFrameDec *d, const float *z, float za, double pos, WSFrameResult *res);
/** Symbol the held lock predicts for the slot pushed back slots ago, or -1. */
int  ff_ws_framedec_expected(const WSFrameDec *d, int back);

/* --- video -------------------------------------------------------------
 *
 * The same 36-bit frame, ids, codes and key carried in the picture, by the
 * vwaterstamp / vwaterdetect filters. The 1023-chip code is laid out on a
 * 33 x 31 grid of cells over the active picture in normalised coordinates,
 * one chip per cell, so scaling keeps it aligned. Within a cell the chip is
 * multiplied by a balanced 4 x 4 Walsh carrier: the cell's own content then
 * cancels exactly when the detector demodulates, and the carrier gives
 * pixel-scale processing gain. Layer A cycles through WSV_CHIPS cyclic
 * shifts per 500 ms slot (one per WSV_CHIP_US), layer B holds the slot's
 * symbol shift plus the same per-chip advance, so that both layers change
 * every chip and a detector can cancel static picture content by
 * subtracting a running mean of the demodulated cells. */
#define WSV_COLS    33
#define WSV_ROWS    31                  /* 33 x 31 = 1023 cells, one chip each */
#define WSV_CELLS   (WSV_COLS * WSV_ROWS)
#define WSV_SUB     4                   /* carrier sub-cells per cell side     */
#define WSV_SUBCOLS (WSV_COLS * WSV_SUB)
#define WSV_SUBROWS (WSV_ROWS * WSV_SUB)
#define WSV_CHIPS   32                  /* layer A chips per slot, 15.625 ms   */
#define WSV_CHIP_US (WS_SLOT_US / WSV_CHIPS)
#define WSV_A_STEP  32                  /* cells of cyclic shift per chip      */

static inline uint32_t ws_hash32(uint32_t h)
{
    h ^= h >> 16; h *= 0x7feb352du;
    h ^= h >> 15; h *= 0x846ca68bu;
    h ^= h >> 16;
    return h;
}

/* Balanced +/-1 carrier for sub-cell (sx, sy) of cell c: one of the nine
 * 4 x 4 Walsh patterns that are non-DC along both axes, chosen per cell.
 * Zero-mean within every cell, and zero-mean along every row and column
 * of it, so DC and linear gradients of the picture cancel exactly. */
static inline int ws_carrier(int c, int sx, int sy)
{
    static const int8_t w[4][4] = { { 1, 1, 1, 1 }, { 1, 1, -1, -1 },
                                    { 1, -1, -1, 1 }, { 1, -1, 1, -1 } };
    unsigned p = ws_hash32(c * 2654435761u) % 9;
    return w[1 + p / 3][sy] * w[1 + p % 3][sx];
}

/* Fixed spatial dither in [0,1) for sub-LSB amplitudes. */
static inline float ws_dither(int x, int y)
{
    return (ws_hash32(x * 73856093u ^ y * 19349663u) & 0xffffff) * (1.0f / 0x1000000);
}

#endif /* AVFILTER_WATERSTAMP_H */

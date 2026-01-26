/*
 * Shared file cache protocol.
 * Copyright (c) 2026 Niklas Haas
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
 *
 * Based on cache.c by Michael Niedermayer
 */

#include "libavutil/avassert.h"
#include "libavutil/avstring.h"
#include "libavutil/hash.h"
#include "libavutil/file_open.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"

#include "url.h"

#include <errno.h>
#include <fcntl.h>
#include <stdatomic.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

/**
 * This hash should be resistant against collision attacks, so that an
 * attacker could not generate e.g. two different URIs that map to the same
 * cache file. This requires at least 64 bits of collision resistance in
 * practice (i.e. 128 bits = 16 bytes of hash size). However, we can be
 * conservative by computing e.g. a 256 bit hash and storing it inside the
 * file header for verification.
 *
 * Note that due to the way we use atomics, we should avoid zero bytes in
 * the resulting hash; hence we tweak the input slightly to avoid this.
 * The resulting loss in hash strength is negligible, since 32 bytes is
 * already much more than needed.
 */
#define HASH_METHOD    "SHA512/256"
#define HASH_SIZE      32

static int hash_uri(uint8_t hash[HASH_SIZE], const char *uri)
{
    struct AVHashContext *ctx = NULL;
    int ret = av_hash_alloc(&ctx, HASH_METHOD);
    if (ret < 0)
        return ret;

    av_assert0(av_hash_get_size(ctx) == HASH_SIZE);
    av_hash_init(ctx);
    av_hash_update(ctx, (const uint8_t *) uri, strlen(uri));
    av_hash_final(ctx, hash);
    av_hash_freep(&ctx);

    for (int i = 0; i < HASH_SIZE; i++)
        hash[i] = hash[i] ? hash[i] : ~hash[i]; /* prevent zero bytes */
    return 0;
}

#define HEADER_MAGIC   MKTAG(u'\xFF', 'S', 'h', '$')
#define HEADER_VERSION 1

enum BlockState {
    BLOCK_NONE,
    BLOCK_CACHED,  ///< block was cached successfully
    BLOCK_PENDING, ///< a thread is currently trying to write this block
    BLOCK_FAILED,  ///< the underlying I/O source failed to read this block
};

typedef struct Spacemap {
    atomic_uint header_magic;
    atomic_ushort version;
    atomic_ushort block_shift;
    atomic_ullong filesize; /* byte offset of true EOF, or 0 if unknown */
    atomic_uchar hash[HASH_SIZE]; /* hash of resource URI / filename */
    char reserved[80];

    atomic_uchar blocks[];
} Spacemap;

/* Set to value iff the current value is unset (zero) */
#define DEF_SET_ONCE(ctype, atype)                                              \
    static int set_once_##atype(atomic_##atype *const ptr, const ctype value)   \
    {                                                                           \
        ctype prev = 0;                                                         \
        av_assert1(value != 0);                                                 \
        if (atomic_compare_exchange_strong_explicit(                            \
                ptr, &prev, value, memory_order_acquire, memory_order_relaxed)) \
            return 1;                                                           \
        else if (prev == value)                                                 \
            return 0;                                                           \
        else                                                                    \
            return AVERROR(EINVAL);                                             \
    }

DEF_SET_ONCE(unsigned char,      uchar)
DEF_SET_ONCE(unsigned int,       uint)
DEF_SET_ONCE(unsigned short,     ushort)
DEF_SET_ONCE(unsigned long long, ullong)

typedef struct SharedContext {
    AVClass *class;
    URLContext *inner;
    int64_t inner_pos;

    /* options */
    char *cache_dir;
    int block_shift; ///< requested shift; may disagree with actual
    int read_only;
    int64_t timeout;
    int retry_errors;

    /* misc state */
    int64_t pos; ///< current logical position
    uint8_t *tmp_buf;
    int block_size;
    int write_err; ///< write error occurred

    /* cache file */
    char *cache_path;
    int fd;

    /* space map */
    Spacemap *spacemap;
    char *map_path;
    off_t map_size;
    int mapfd;

    /* statistics */
    int64_t nb_hit;
    int64_t nb_miss;
} SharedContext;

static int shared_close(URLContext *h)
{
    SharedContext *s = h->priv_data;

    ffurl_close(s->inner);
    if (s->spacemap)
        munmap(s->spacemap, s->map_size);
    if (s->fd != -1)
        close(s->fd);
    if (s->mapfd != -1)
        close(s->mapfd);
    av_freep(&s->cache_path);
    av_freep(&s->map_path);
    av_freep(&s->tmp_buf);

    av_log(h, AV_LOG_DEBUG, "Cache statistics: %"PRId64" hits, %"PRId64" misses\n",
           s->nb_hit, s->nb_miss);
    return 0;
}

static int spacemap_init(URLContext *h, const uint8_t hash[HASH_SIZE]);
static int spacemap_grow(URLContext *h, int64_t block);

static int64_t get_filesize(URLContext *h)
{
    SharedContext *s = h->priv_data;
    return atomic_load_explicit(&s->spacemap->filesize, memory_order_relaxed);
}

static int set_filesize(URLContext *h, int64_t new_size)
{
    SharedContext *s = h->priv_data;
    int ret;

    if (!new_size)
        return 0;

    ret = set_once_ullong(&s->spacemap->filesize, new_size);
    if (ret < 0) {
        av_log(h, AV_LOG_ERROR, "Cached file size mismatch, expected: "
                "%"PRId64", got: %"PRIu64"!\n", new_size,
                (uint64_t) atomic_load(&s->spacemap->filesize));
        return ret;
    } else if (ret) {
        /* Opportunistically set the filesize metadata, ignoring failure as
         * this is not relevant to the cache logic */
        ftruncate(s->fd, new_size);
    }

    return ret;
}

static int shared_open(URLContext *h, const char *arg, int flags, AVDictionary **options)
{
    SharedContext *s = h->priv_data;
    int ret;

    if (!s->cache_dir || !s->cache_dir[0]) {
        av_log(h, AV_LOG_ERROR, "Missing path for shared cache!\n");
        return AVERROR(EINVAL);
    }

    s->fd = s->mapfd = -1; /* Set these early for shared_close() failure path */

    /* Open underlying protocol */
    av_strstart(arg, "shared:", &arg);
    ret = ffurl_open_whitelist(&s->inner, arg, flags, &h->interrupt_callback,
                               options, h->protocol_whitelist, h->protocol_blacklist, h);

    if (ret < 0)
        goto fail;

    uint8_t hash[HASH_SIZE];
    ret = hash_uri(hash, arg);
    if (ret < 0)
        goto fail;

    /* 128 bits is enough for collision resistance; we already store the full
     * hash inside the header for verification */
    char filename[2 * 16 + 1];
    for (int i = 0; i < FF_ARRAY_ELEMS(filename) / 2; i++)
        sprintf(&filename[i * 2], "%02X", hash[i]);
    s->cache_path = av_asprintf("%s/%s.cache",    s->cache_dir, filename);
    s->map_path   = av_asprintf("%s/%s.spacemap", s->cache_dir, filename);
    if (!s->cache_path || !s->map_path) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    av_log(h, AV_LOG_VERBOSE, "Opening cache file '%s' for URI: '%s'\n",
           s->cache_path, s->inner->filename);

    s->fd    = avpriv_open(s->cache_path, O_RDWR | O_CREAT, 0660);
    s->mapfd = avpriv_open(s->map_path,   O_RDWR | O_CREAT, 0660);
    if (s->fd < 0 || s->mapfd < 0) {
        ret = AVERROR(errno);
        av_log(h, AV_LOG_ERROR, "Failed to open '%s': %s\n",
               s->fd < 0 ? s->cache_path : s->map_path, av_err2str(ret));
        goto fail;
    }

    ret = spacemap_init(h, hash);
    if (ret < 0)
        goto fail;

    s->block_size = 1 << atomic_load(&s->spacemap->block_shift);
    s->tmp_buf = av_malloc(s->block_size);
    if (!s->tmp_buf) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    int64_t filesize = get_filesize(h);
    if (!filesize) {
        /* Filesize is not yet known, try to get it from the underlying URL */
        filesize = ffurl_size(s->inner);
        if (ret < 0 && ret != AVERROR(ENOSYS))
            goto fail;
        else if (ret > 0)
            set_filesize(h, filesize);
    }

    if (filesize > 0) {
        int64_t last_pos = filesize;
        int64_t last_block = last_pos >> atomic_load(&s->spacemap->block_shift);
        ret = spacemap_grow(h, last_block);
        if (ret < 0)
            return ret;
    }

    h->max_packet_size = s->block_size;
    h->min_packet_size = s->block_size;

fail:
    if (ret < 0)
        shared_close(h);
    return ret;
}

static int spacemap_remap(URLContext *h, size_t map_size)
{
    SharedContext *s = h->priv_data;
    struct flock fl = { .l_type  = F_WRLCK };
    int ret, did_grow = 0;
    if (map_size <= s->map_size)
        return 0;

    /* Opportunistically get current filesize before attempting to lock */
    struct stat st;
    ret = fstat(s->mapfd, &st);
    if (ret < 0) {
        ret = AVERROR(errno);
        goto fail;
    }

    if (st.st_size >= map_size)
        goto skip_resize;

    /* Lock the spacemap to ensure nobody else is currently resizing it */
    ret = fcntl(s->mapfd, F_SETLKW, &fl);
    if (ret < 0) {
        ret = AVERROR(errno);
        goto fail;
    }
    fl.l_type = F_UNLCK;

    /* Refresh filesize after acquiring the lock */
    ret = fstat(s->mapfd, &st);
    if (ret < 0) {
        ret = AVERROR(errno);
        goto fail;
    }

    if (st.st_size >= map_size)
        goto skip_resize;

    ret = ftruncate(s->mapfd, map_size);
    if (ret < 0)
        goto fail;
    st.st_size = map_size;
    did_grow = 1;

skip_resize:
    if (s->spacemap)
        munmap(s->spacemap, s->map_size);
    s->map_size = st.st_size;
    s->spacemap = mmap(NULL, s->map_size, PROT_READ | PROT_WRITE, MAP_SHARED, s->mapfd, 0);
    if (s->spacemap == MAP_FAILED) {
        s->spacemap = NULL; /* for munmap check */
        s->map_size = 0;
        ret = AVERROR(errno);
        goto fail;
    }

    /* fl.l_type is set to F_UNLCK only after successful lock */
    if (fl.l_type == F_UNLCK)
        fcntl(s->mapfd, F_SETLK, &fl);

    return did_grow;

fail:
    if (fl.l_type == F_UNLCK)
        fcntl(s->mapfd, F_SETLK, &fl);
    av_log(h, AV_LOG_ERROR, "Failed to resize space map: %s\n", av_err2str(ret));
    return ret;
}

static int spacemap_grow(URLContext *h, int64_t block)
{
    SharedContext *s = h->priv_data;
    int64_t num_blocks = block + 1;
    size_t map_bytes = sizeof(Spacemap) + num_blocks;

    /* When streaming files without known size, round up the number of blocks
     * to the nearest multiple of the block size to reduce the rate of resizes */
    if (!get_filesize(h)) {
        av_assert0(s->block_size > 0);
        map_bytes = FFALIGN(map_bytes, (int64_t) s->block_size);
    }

    if (map_bytes < num_blocks)
        return AVERROR(EINVAL); /* overflow */

    int ret = spacemap_remap(h, map_bytes);
    if (ret < 0)
        return ret;

    /* Report new size after successful grow */
    num_blocks = s->map_size - sizeof(Spacemap);
    av_log(h, AV_LOG_DEBUG, "%s %zu bytes, capacity: %"PRId64" blocks = %zu MB\n",
           ret ? "Resized spacemap to" : "Mapped spacemap with",
           (size_t) s->map_size, num_blocks,
           (num_blocks * (int64_t) s->block_size) >> 20);
    return 0;
}

static int spacemap_init(URLContext *h, const uint8_t hash[HASH_SIZE])
{
    SharedContext *s = h->priv_data;
    int ret;

    ret = spacemap_remap(h, sizeof(Spacemap));
    if (ret < 0)
        return ret;

    if ((ret = set_once_uint(&s->spacemap->header_magic, HEADER_MAGIC)) < 0 ||
        (ret = set_once_ushort(&s->spacemap->version, HEADER_VERSION)) < 0)
    {
        av_log(h, AV_LOG_ERROR, "Shared cache spacemap header mismatch!\n");
        av_log(h, AV_LOG_ERROR, "  Expected magic: 0x%X, version: %d\n",
               HEADER_MAGIC, HEADER_VERSION);
        av_log(h, AV_LOG_ERROR, "  Got      magic: 0x%X, version: %d\n",
               atomic_load(&s->spacemap->header_magic),
               atomic_load(&s->spacemap->version));
        return ret;
    }

    ret = set_once_ushort(&s->spacemap->block_shift, s->block_shift);
    if (ret < 0) {
        const int shift = atomic_load(&s->spacemap->block_shift);
        av_log(h, AV_LOG_WARNING, "Shared cache uses block shift %d, "
               "but requested block shift is %d.\n", shift, s->block_shift);
        if (shift < 9 || shift > 16) {
            av_log(h, AV_LOG_ERROR, "Invalid block shift %d in cache file!\n", shift);
            return AVERROR(EINVAL);
        }
    }

    for (int i = 0; i < HASH_SIZE; i++) {
        ret = set_once_uchar(&s->spacemap->hash[i], hash[i]);
        if (ret < 0) {
            av_log(h, AV_LOG_ERROR, "Shared cache spacemap hash mismatch!\n");
            av_log(h, AV_LOG_ERROR, "  Expected hash: ");
            for (int j = 0; j < 32; j++)
                av_log(h, AV_LOG_ERROR, "%02X", hash[j]);
            av_log(h, AV_LOG_ERROR, "\n  Got      hash: ");
            for (int j = 0; j < 32; j++)
                av_log(h, AV_LOG_ERROR, "%02X", atomic_load(&s->spacemap->hash[j]));
            av_log(h, AV_LOG_ERROR, "\n");
            return ret;
        }
    }

    if (ret) /* set_once() return 1 if this is the first time setting the value */
        av_log(h, AV_LOG_DEBUG, "Initialized new cache spacemap.\n");

    return ret;
}

static ssize_t pwrite_loop(int fd, const uint8_t *buf, size_t size, off_t offset)
{
    const size_t total = size;
    while (size) {
        ssize_t ret = pwrite(fd, buf, size, offset);
        if (ret < 0)
            return ret;
        buf    += ret;
        offset += ret;
        size   -= ret;
    }
    return total;
}

static int shared_read(URLContext *h, unsigned char *buf, int size)
{
    SharedContext *s = h->priv_data;
    int ret;

    if (size <= 0)
        return 0;

    int64_t filesize = get_filesize(h);
    if (filesize) { /* limit read request to true filesize if known */
        if (s->pos + size >= filesize)
            size = filesize - s->pos;
        if (size <= 0)
            return AVERROR_EOF;
    }

    const int shift = atomic_load_explicit(&s->spacemap->block_shift, memory_order_relaxed);
    const int64_t block = s->pos >> shift;
    const int64_t offset = s->pos & (s->block_size - 1);
    const int64_t block_pos = block * s->block_size;
    ret = spacemap_grow(h, block);
    if (ret < 0)
        return ret;

    atomic_uchar *const block_state = &s->spacemap->blocks[block];
    unsigned char state = atomic_load_explicit(block_state, memory_order_acquire);
    int64_t pending_since = 0;

retry:
    switch (state) {
    case BLOCK_CACHED:
        size = FFMIN(size, s->block_size - offset);
        ret = pread(s->fd, buf, size, s->pos);
        if (ret < 0) {
            ret = AVERROR(errno);
            av_log(h, AV_LOG_ERROR, "Failed to read from cache file: %s\n", av_err2str(ret));
            return ret;
        }

        s->nb_hit++;
        s->pos += ret;
        return ret;

    case BLOCK_FAILED:
        if (!s->retry_errors)
            return AVERROR(EIO);
        /* fall through */
    case BLOCK_NONE:
        if (atomic_compare_exchange_weak_explicit(block_state, &state,
                                                  BLOCK_PENDING,
                                                  memory_order_acquire,
                                                  memory_order_acquire))
        {
            /* Acquired pending state, proceed to fetch the block */
            state = BLOCK_PENDING;
            break;
        }
        /* CAS failed, another thread changed the state; reload it */
        goto retry;

    case BLOCK_PENDING:
        /* Another thread is busy fetching this block, wait for it to finish */
        if (!s->timeout) {
            break; /* no timeout requested, immediately race to fetch block */
        } else if (pending_since) {
            int64_t new = av_gettime_relative();
            if (new - pending_since >= s->timeout)
                break; /* timeout expired, try to fetch the block ourselves */
        } else {
            pending_since = av_gettime_relative();
        }

        /* Make sure we try a few times before giving up */
        av_usleep(s->timeout >> 4);
        state = atomic_load_explicit(block_state, memory_order_acquire);
        goto retry;
    }

    /* Cache miss, fetch this block from underlying protocol */
    s->nb_miss++;

    const int read_only = s->read_only || s->write_err;
    int64_t inner_pos = read_only ? s->pos : block_pos;
    if (s->inner_pos != inner_pos) {
        inner_pos = ffurl_seek(s->inner, inner_pos, SEEK_SET);
        if (inner_pos < 0) {
            av_log(h, AV_LOG_ERROR, "Failed to seek underlying protocol: %s\n",
                   av_err2str(inner_pos));
            return inner_pos;
        }

        s->inner_pos = inner_pos;
    }

    if (read_only) {
        /* Directly defer to the underlying protocol */
        ret = ffurl_read(s->inner, buf, size);
        if (ret >= 0)
            s->pos = s->inner_pos = inner_pos + ret;
        return ret;
    }

    /* Try and fetch the entire block; reuse the output buffer if possible */
    const int block_size = filesize ? FFMIN(filesize - inner_pos, s->block_size) : s->block_size;
    uint8_t *const tmp = (size >= block_size && !offset) ? buf : s->tmp_buf;
    int bytes_read = 0;
    while (bytes_read < block_size) {
        ret = ffurl_read(s->inner, &tmp[bytes_read], block_size - bytes_read);
        if (!ret || ret == AVERROR_EOF)
            break;
        else if (ret < 0) {
            /* Try to mark block as failed; ignore errors - any mismatch
             * here will mean that either another thread already marked it
             * as failed, or successfully cached it in the meantime */
            atomic_compare_exchange_strong_explicit(block_state, &state,
                                                    BLOCK_FAILED,
                                                    memory_order_relaxed,
                                                    memory_order_relaxed);
            return ret;
        }

        bytes_read += ret;
        s->inner_pos += ret;
    }

    if (bytes_read < block_size) {
        /* Learned location of true EOF, update filesize */
        ret = set_filesize(h, inner_pos + bytes_read);
        if (ret < 0)
            return ret;
    }

    if (bytes_read > 0) {
        ret = pwrite_loop(s->fd, tmp, bytes_read, inner_pos);
        if (ret == bytes_read) {
            atomic_store_explicit(block_state, BLOCK_CACHED, memory_order_release);
        } else {
            av_log(h, AV_LOG_ERROR, "Failed to write to cache file: %s\n",
                   av_err2str(AVERROR(errno)));
            s->write_err = 1;
        }
    } else {
        return AVERROR_EOF;
    }

    const int wanted = FFMIN(bytes_read - offset, size);
    av_assert0(wanted >= 0);
    if (tmp != buf)
        memcpy(buf, &s->tmp_buf[offset], wanted);
    s->pos += wanted;
    return wanted;
}

static int64_t shared_seek(URLContext *h, int64_t pos, int whence)
{
    SharedContext *s = h->priv_data;
    const int64_t filesize = get_filesize(h);

    if (whence == SEEK_SET) {
        return s->pos = pos;
    } else if (whence == SEEK_CUR) {
        return s->pos += pos;
    } else if (whence == SEEK_END) {
        if (filesize)
            return s->pos = filesize + pos;
        /* Defer to underlying protocol if filesize is unknown */
        int64_t res = ffurl_seek(s->inner, pos, whence);
        if (res < 0)
            return res;
        set_filesize(h, res - pos); /* Opportunistically update known filesize */
        return s->pos = s->inner_pos = res;
    } else if (whence == AVSEEK_SIZE) {
        if (filesize)
            return filesize;
        int64_t res = ffurl_seek(s->inner, pos, whence);
        if (res < 0)
            return res;
        set_filesize(h, res);
        return res;
    } else {
        return AVERROR(EINVAL);
    }
}

static int shared_get_file_handle(URLContext *h)
{
    SharedContext *s = h->priv_data;
    return ffurl_get_file_handle(s->inner);
}

static int shared_get_short_seek(URLContext *h)
{
    SharedContext *s = h->priv_data;
    int ret = ffurl_get_short_seek(s->inner);
    if (ret < 0)
        return ret;
    return FFMAX(ret, s->block_size);
}

#define OFFSET(x) offsetof(SharedContext, x)
#define D AV_OPT_FLAG_DECODING_PARAM

static const AVOption options[] = {
    { "cache_dir",      "Directory path for shared file cache",             OFFSET(cache_dir),      AV_OPT_TYPE_STRING, {.str = NULL}, .flags = D },
    { "block_shift",    "Set the base 2 logarithm of the block size",       OFFSET(block_shift),    AV_OPT_TYPE_INT, {.i64 = 15}, 9, 30, .flags = D },
    { "read_only",      "Don't write data to the cache, only read from it", OFFSET(read_only),      AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, .flags = D },
    { "timeout",        "Time in us to wait before re-fetching pending blocks", OFFSET(timeout),    AV_OPT_TYPE_INT64, {.i64 = 0}, 0, INT64_MAX, .flags = D },
    { "retry_errors",   "Re-request blocks even if they previously failed", OFFSET(retry_errors),   AV_OPT_TYPE_BOOL, {.i64 = 1}, 0, 1, .flags = D },
    {0},
};

static const AVClass shared_context_class = {
    .class_name = "shared",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const URLProtocol ff_shared_protocol = {
    .name                = "shared",
    .url_open2           = shared_open,
    .url_read            = shared_read,
    .url_seek            = shared_seek,
    .url_close           = shared_close,
    .url_get_file_handle = shared_get_file_handle,
    .url_get_short_seek  = shared_get_short_seek,
    .priv_data_size      = sizeof(SharedContext),
    .priv_data_class     = &shared_context_class,
};

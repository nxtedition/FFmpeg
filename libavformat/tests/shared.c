/*
 * Shared cache byte-range tests.
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

#include "config.h"

#include <stdio.h>
#include <sys/stat.h>
#if HAVE_UNISTD_H
#include <unistd.h>
#endif

#include "libavutil/dict.h"
#include "libavutil/error.h"
#include "libavutil/log.h"
#include "libavutil/random_seed.h"
#include "libavformat/avio.h"
#include "libavformat/os_support.h"
#include "libavformat/url.h"

#define FILE_SIZE 2048

static int check_read(const char *url, const char *directory,
                      int offset, int end_offset, int read_only)
{
    URLContext *context = NULL;
    AVDictionary *options = NULL;
    unsigned char buffer[1024];
    int position = offset, end = end_offset ? end_offset : FILE_SIZE;
    int ret;

    av_dict_set(&options, "cache_dir", directory, 0);
    av_dict_set(&options, "block_shift", "9", 0);
    av_dict_set_int(&options, "offset", offset, 0);
    av_dict_set_int(&options, "end_offset", end_offset, 0);
    av_dict_set_int(&options, "cache_read_only", read_only, 0);
    ret = ffurl_open_whitelist(&context, url, AVIO_FLAG_READ, NULL,
                               &options, NULL, NULL, NULL);
    av_dict_free(&options);
    if (ret < 0)
        return 1;

    if (ffurl_size(context) != FILE_SIZE) {
        fprintf(stderr, "reader range changed the resource size\n");
        ret = 1;
        goto done;
    }
    while ((ret = ffurl_read(context, buffer, sizeof(buffer))) > 0) {
        if (position + ret > end) {
            fprintf(stderr, "read past end_offset: %d + %d > %d\n", position, ret, end);
            ret = 1;
            goto done;
        }
        for (int i = 0; i < ret; i++) {
            if (buffer[i] != (unsigned char)(position + i)) {
                fprintf(stderr, "incorrect byte at %d\n", position + i);
                ret = 1;
                goto done;
            }
        }
        position += ret;
    }
    ret = ret != AVERROR_EOF || position != end;
    if (ret)
        fprintf(stderr, "unexpected EOF at %d, expected %d\n", position, end);

done:
    ffurl_closep(&context);
    return ret;
}

int main(void)
{
    char directory[64], source[96], url[128], entry_path[256];
    AVIODirContext *dir = NULL;
    AVIODirEntry *entry = NULL;
    FILE *file;
    int ret = 1;

    av_log_set_level(AV_LOG_ERROR);
    snprintf(directory, sizeof(directory), "ff-shared-test-%08x", av_get_random_seed());
    snprintf(source, sizeof(source), "%s/source", directory);
    snprintf(url, sizeof(url), "shared:file:%s", source);
    if (mkdir(directory, 0700) < 0)
        return 1;
    file = fopen(source, "wb");
    if (!file)
        goto cleanup;
    for (int i = 0; i < FILE_SIZE; i++)
        fputc((unsigned char)i, file);
    if (fclose(file))
        goto cleanup;

    /* An unaligned cold range must fill a complete cache block. The same
     * range and an overlapping one must also stop on a warm cache. */
    if (check_read(url, directory, 37, 123, 0) ||
        check_read(url, directory, 37, 123, 0) ||
        check_read(url, directory, 101, 400, 0) ||
        check_read(url, directory, 123, 123, 0) ||
        check_read(url, directory, 0, 0, 0) ||
        check_read(url, directory, 0, 2047, 1))
        goto cleanup;
    ret = 0;

cleanup:
    if (avio_open_dir(&dir, directory, NULL) >= 0) {
        while (avio_read_dir(dir, &entry) >= 0 && entry) {
            snprintf(entry_path, sizeof(entry_path), "%s/%s", directory, entry->name);
            avio_free_directory_entry(&entry);
            unlink(entry_path);
        }
        avio_close_dir(&dir);
    }
    rmdir(directory);
    return ret;
}

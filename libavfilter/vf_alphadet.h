#ifndef AVFILTER_ALPHADET_H
#define AVFILTER_ALPHADET_H

#include "libavutil/pixdesc.h"
#include "avfilter.h"

typedef enum {
    NONE,
    STRAIGHT,
    UNDETERMINED,
} Type;

typedef struct ALPHADETContext {
    const AVClass *class;
    Type type;
    const AVPixFmtDescriptor *csp;
    void *range_lut;
} ALPHADETContext;

#endif
